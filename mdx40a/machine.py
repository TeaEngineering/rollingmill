"""
Roland MDX-40A machine interface.

Single-threaded: the caller (typically the TUI curses main loop) drives state
polling by calling MDX40A.poll() on a cadence (recommended POLL_INTERVAL).
poll() reads the state block, fires registered observers, and refreshes the
spindle-time counter once per minute.  All USB interaction goes through
mdx40a.usb primitives — no direct libusb calls here.
"""

import logging
import struct
import time
from dataclasses import dataclass, field
from typing import Optional

import usb.core

from . import usb as _usb

log = logging.getLogger(__name__)

POLL_INTERVAL = 0.200   # seconds — recommended cadence for caller of MDX40A.poll()

# Firmware-max jog speed (mm/min). VPanel sends 0xFFFF for single-press steps.
# RE: DAT_00440aa0=240 (ramp start), DAT_00440aac=8000 (XYZ ramp max).
JOG_SPEED_MAX = 0xFFFF  # firmware maximum

# Spindle RPM limits (MDX-40A hardware range: 4500–15000 RPM)
# RE: FUN_00402910 constructor sets param_1[0x30]=15000; min confirmed from
# state-block observations (idle firmware reports 0x1194 = 4500 RPM).
SPINDLE_RPM_MIN =  4500
SPINDLE_RPM_MAX = 15000

# WCS slot Pattern B read wValues (slots 1-10, 0-indexed in tuple)
# RE: query_coord_system_by_index @ 0x00403910 → GET wValue returns 4×uint32 XYZA
_WCS_READ_WVAL = (
    0x030b,                              # WCS1
    0x3202,                              # WCS2
    0x3203, 0x3204, 0x3205,             # WCS3-5
    0x3206, 0x3207, 0x3208,             # WCS6-8
    0x3209, 0x320a,                      # WCS9-10
)

# WCS slot explicit write wValues (slots 1-10, 0-indexed in tuple)
# RE: FUN_00403a40, called from Detect Jig and origin-set sequences
_WCS_WRITE_WVAL = (
    0x030c,                              # WCS1
    0x3335,                              # WCS2
    0x3336, 0x3337, 0x3338,             # WCS3-5
    0x3339, 0x333a, 0x333b,             # WCS6-8
    0x333c, 0x333d,                      # WCS9-10
)

# Tool diameter offset wValues (slots 1-8, 0-indexed in tuple)
# RE: FUN_00402990 (read) / apply_axis_config_to_device (write)
_TOOL_OFFSET_READ_WVAL  = tuple(0x346a + i for i in range(1, 9))   # 0x346b..0x3472
_TOOL_OFFSET_WRITE_WVAL = tuple(0x347a + i for i in range(1, 9))   # 0x347b..0x3482

# Ping status bits (GET wValue=0x0001, 4-byte response, little-endian uint32)
# RE: jog_wait_busy_bits_clear @ 0x00417b00, wait_move_bit_clear @ 0x0041b8d0
_PING_BUSY_MASK  = 0x00200004   # bits 21 and 2 — firmware busy (jog motion-complete gate)
_PING_ERROR_MASK = 0x00100000   # bit 20
_PING_BIT21      = 0x00200000   # bit 21 alone — "command acknowledged" flag polled after
                                 # SET 0x3901 (spindle RPM), SET 0x2425 (reset time), etc.

# ── Machine status flags (first 4 bytes of wValue=0x0100 block, big-endian) ──
#
# Decoded from VP_MDX40A.exe update_state_and_coords + wait_motion_complete_loop.
# Byte ordering: Python struct '>I' gives the same value as the display.
#
#  bit 28  0x10000000  DOOR       enclosure door open (jog inhibited while set)
#  bit 27  0x08000000  SPINDLE    spindle motor on  (fires RPM/speed update)
#  bit 26  0x04000000  CMD_MOVE   motion command executing
#  bit 25  0x02000000  TOOLBUTTON using tool button on front pannel
#  bit 22  0x00400000  MOVING     axis velocity > 0 (actually translating)
#  18-16   0x00070000  STATE      machine state enum
#                                   2 = idle / normal
#                                   3 = motion commanded (triggers origin update on→2)
#  bit 13  0x00002000  (always set in observed states — semantic unclear; do NOT
#                       gate motion completion on this. VPanel's
#                       jog_wait_busy_bits_clear @ 0x00417b00 actually polls
#                       ping word bits 2 and 21, not this state-block bit.)
#  bit 12  0x00001000  ERROR      error condition
#  bit 23  0x00800000  ]
#  bit 17  0x00020000  ] constant in all observed states — likely axis-present or
#  bit 11  0x00000800  ] hw-config flags set at power-on; ignore for status display
#  bit  4  0x00000010  ]
#  bit  3  0x00000008  ]
#  bit  2  0x00000004  ]

FLAG_DOOR     = 0x10000000
FLAG_SPINDLE  = 0x08000000
FLAG_CMD_MOVE = 0x04000000
FLAG_TOOLBTN  = 0x02000000
FLAG_MOVING   = 0x00400000
FLAG_BUSY     = 0x00002000
FLAG_ERROR    = 0x00001000
FLAG_STATE    = 0x00070000
FLAG_STATE_SHIFT = 16

STATE_MAP = {0: 'init', 1: 'homing', 2: 'idle', 3: 'moving', 4: 'error'}

@dataclass
class MachineState:
    flags:       int   = 0
    x_mm:        float = 0.0    # machine coords, mm
    y_mm:        float = 0.0
    z_mm:        float = 0.0
    a_deg:       float = 0.0
    spindle_rpm: int   = 0      # firmware target RPM (state block bytes 20-23)
    raw:         bytes = field(default_factory=bytes, repr=False)

    @property
    def ready(self) -> bool:
        """True when the state block has been populated from the device."""
        return bool(self.raw)

    @property
    def idle(self) -> bool:
        """True when the firmware has finished init/homing and is at idle (STATE == 2).

        Safe gate for follow-up queries (spindle time, rotary centreline, etc.)
        that rely on firmware state only populated once the machine reaches idle.
        """
        return self.ready and (self.flags & FLAG_STATE) >> FLAG_STATE_SHIFT == 2


def _decode_state(data: bytes) -> MachineState:
    data = bytes(data)
    flags = struct.unpack_from('>I', data, 0)[0]
    x, y, z, a = struct.unpack_from('>4i', data, 4)
    spindle_rpm = struct.unpack_from('>I', data, 20)[0] if len(data) >= 24 else 0
    return MachineState(
        flags=flags,
        x_mm=x / 1000.0,
        y_mm=y / 1000.0,
        z_mm=z / 1000.0,
        a_deg=a / 1000.0,
        spindle_rpm=spindle_rpm,
        raw=data,
    )


class MDX40A:
    """
    High-level interface to the Roland MDX-40A.

    Usage::

        machine = MDX40A()
        machine.connect()
        machine.poll()            # refresh state and ping caches
        machine.send_jog('Z', -2.0)   # move 2 mm down (fire-and-forget)
        machine.release()
    """

    def __init__(self):
        self._dev = None
        self._intf_num: Optional[int] = None
        self._state = MachineState()
        self._last_ping_word: Optional[int] = None   # most recent GET 0x0001 (uint32 LE); see is_busy
        self._spindle_last_read_at: float = 0.0
        self._spindle_speed_pct: int = 100   # cached spindle override %
        self._cutting_feed_pct: int = 100    # cached cutting feed override %
        self._spindle_range: Optional[tuple] = None          # (speed_min, speed_max) uint32 pair from GET 0x3005
        self._spindle_live_speed: Optional[int] = None       # latest GET 0x3003 uint32 — current spindle/feed speed
        self._spindle_target_rpm: int = SPINDLE_RPM_MIN  # configured target RPM (GET/SET 0x3900/0x3901)
        self._spindle_secs: Optional[int] = None
        self._active_wcs: int   = 0                      # 0=MCS, 1-10=WCS1-WCS10
        self._wcs_offset: tuple = (0.0, 0.0, 0.0, 0.0)  # machine coords of active WCS origin (mm/deg)
        self._rotary_extension_byte: Optional[int] = None    # latest GET 0x3800 byte (0=none, 1=rotary, 2=rotary+vice)
        self._rotary_centerline: Optional[tuple] = None      # stored A-axis centreline (x_mm, y_mm, z_mm); read once at idle
        self._firmware_id: Optional[str] = None              # ASCII model/firmware string from GET 0x0101 (read at handshake)
        self._axis_scaling_pct: Optional[tuple] = None       # (X%, Y%, Z%, A%) from GET 0x5f0; static, read at handshake
        self._rotary_angle_correction: Optional[tuple] = None  # (refX1, ofsY1, ofsZ1, refX2, ofsY2, ofsZ2) mm from GET 0x3804

    # ── Connection lifecycle ──────────────────────────────────────────────────

    def connect(self) -> None:
        """Find and open the MDX-40A.  Raises RuntimeError if not found."""
        dev = _usb.find_device()
        if dev is None:
            raise RuntimeError(
                "MDX-40A not found (VID=0x0B75 PID=0x03DB). Is it powered on?"
            )
        log.info(
            "Found %r %r  bus=%d addr=%d serial=%r",
            dev.manufacturer, dev.product, dev.bus, dev.address, dev.serial_number,
        )
        self._dev = dev
        self._intf_num = _usb.claim(dev)
        log.debug("Claimed interface %d", self._intf_num)
        self._handshake()

    def release(self) -> None:
        """Release the USB interface."""
        if self._dev is not None and self._intf_num is not None:
            _usb.release(self._dev, self._intf_num)
            log.info("Interface %d released", self._intf_num)
        self._dev = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.release()

    # ── State access ─────────────────────────────────────────────────────────

    @property
    def state(self) -> MachineState:
        return self._state

    # ── Poll handoff ──────────────────────────────────────────────────────────

    def poll(self) -> Optional[MachineState]:
        """Service one poll tick — refresh state block + ping word, refresh
        spindle time periodically.

        Caller is expected to invoke this on a cadence (~POLL_INTERVAL seconds).
        Two USB reads happen each tick:
          - _read_state()  : GET 0x0100, caches MachineState on self._state.
          - _ping_status() : GET 0x0001, caches uint32 on self._last_ping_word.
            The ping word drives jog-end detection (TUI consults self.is_busy,
            which derives from the cached value).
        """
        s = self._read_state()
        self._ping_status()
        machine_idle = s is not None and s.idle
        now = time.monotonic()
        if machine_idle and now - self._spindle_last_read_at >= self._SPINDLE_POLL_INTERVAL:
            self._spindle_last_read_at = now
            secs = self.get_spindle_time()
            if secs is not None:
                self._spindle_secs = secs
        # Read rotary centreline lazily on first idle state — calibration value, static at runtime.
        if self._rotary_centerline is None and machine_idle:
            self._rotary_centerline = self.get_rotary_axis_centreline()
        return s

    # ── Jog ──────────────────────────────────────────────────────────────────

    def send_jog(
        self,
        axis: str,
        dist_mm: float,
        speed: int = 240,
    ) -> None:
        """Send a relative-displacement jog and return immediately.

        Fire-and-forget: the caller observes completion via the regular poll()
        cycle (state flags FLAG_MOVING / FLAG_CMD_MOVE).
        axis: 'X', 'Y', 'Z', or 'A' (A in degrees).
        """
        axis = axis.upper()
        if axis not in ('X', 'Y', 'Z', 'A'):
            raise ValueError(f"axis must be X/Y/Z/A, got {axis!r}")
        # Relative displacement vector (1/1000 mm units). wValue=0x4f5 takes a
        # displacement, not an absolute target — only the jogged axis is non-zero.
        delta = round(dist_mm * 1000)
        dx = delta if axis == 'X' else 0
        dy = delta if axis == 'Y' else 0
        dz = delta if axis == 'Z' else 0
        da = delta if axis == 'A' else 0
        log.info(
            "Jog %s %+.3f mm  delta=[%d,%d,%d,%d]  speed=%d",
            axis, dist_mm, dx, dy, dz, da, speed,
        )
        self._send_jog(speed, dx, dy, dz, da)

    def _set_operation_mode(self, active: bool) -> None:
        """SET 0x1109 operation bracket (RE: FUN_0041c410).

        VPanel sends 0x00 before any interactive operation (jog, detect jig, move-to)
        and 0xff after. Hypothesis: releases the firmware parking brake / enables
        servo drive for remote commands.
        """
        payload = b'\x00' if active else b'\xff'
        try:
            _usb.vend_set(self._dev, 0x1109, payload)
            log.debug("Operation mode: %s", "begin (0x00)" if active else "end (0xff)")
        except usb.core.USBError as e:
            log.warning("SET 0x1109 failed: %s", e)

    def _send_jog(self, speed: int, x: int, y: int, z: int, a: int) -> None:
        """Send wValue=0x04f5 displacement jog (RE: FUN_00419e10 in VP_MDX40A.exe).

        0x4f5 takes a relative displacement vector [X,Y,Z,A] in 1/1000 mm units.
        The second uint16 field must be 0x0000 (0xFFFF is for wValue=0x4f7 which
        is the absolute-position form used only in multi-step milling sequences).
        """
        payload = struct.pack('>HH4i', speed, 0x0000, x, y, z, a)
        _usb.vend_set(self._dev, 0x04f5, payload)
        log.debug("Jog cmd sent: %s", payload.hex())

    def _ping_status(self) -> Optional[int]:
        """Read the 4-byte ping word (GET wValue=0x0001), cache it on self, return it.

        Single canonical reader for the 0x0001 register — every caller that
        polls it goes through here:
          - _wait_ping_bit21 — command-ack polling (bit 21)
          - get_spindle_time — response-length wait (high byte)
          - poll() — refreshes the cache so the TUI's jog-end detection (which
            reads self.last_ping_word) sees a fresh value each tick.

        The cached value drives the jog-end check: VPanel's
        jog_wait_busy_bits_clear @ 0x00417b00 loops while bits 2 or 21 of this
        word are set, so jog completion = (last_ping_word & 0x00200004) == 0.

        Returns the raw 4 bytes as an integer, as the bytes are not endian swapped,
        or None on USB error / short response.
        """
        try:
            data = _usb.vend_get(self._dev, 0x0001, 4)
            if len(data) < 4:
                return None
            word = struct.unpack_from('<I', data)[0]
            self._last_ping_word = word
            return word
        except usb.core.USBError:
            return None

    @property
    def last_ping_word(self) -> Optional[int]:
        """Most recently read 0x0001 ping word, or None before the first read."""
        return self._last_ping_word

    @property
    def is_busy(self) -> bool:
        """True if the firmware reports motion-in-progress on the latest ping.

        Bits 2 or 21 of GET 0x0001 — the same exit condition VPanel's
        jog_wait_busy_bits_clear polls. Returns False when no ping has been
        received yet (treat unknown as 'not busy').
        """
        p = self._last_ping_word
        return p is not None and (p & _PING_BUSY_MASK) != 0

    def stop_motion(self) -> None:
        """Send immediate motion stop (SET wValue=0x03f3)."""
        try:
            _usb.vend_set(self._dev, 0x03f3)
            log.info("Motion stop sent (SET 0x03f3)")
        except usb.core.USBError as e:
            log.warning("Motion stop failed: %s", e)

    # ── Spindle control ───────────────────────────────────────────────────────

    @property
    def spindle_speed_pct(self) -> int:
        return self._spindle_speed_pct

    @property
    def spindle_target_rpm(self) -> int:
        return self._spindle_target_rpm

    @property
    def spindle_range(self) -> Optional[tuple]:
        """Spindle speed range (speed_min, speed_max) from GET 0x3005.

        Two big-endian uint32s refreshed on every poll. None before the first
        poll completes. The current spindle/feed speed (GET 0x3003) is clamped
        by the firmware to this range.
        """
        return self._spindle_range

    @property
    def spindle_live_speed(self) -> Optional[int]:
        """Current spindle/feed speed from GET 0x3003 (uint32, refreshed every poll).

        Clamped by the firmware to `spindle_range`. None before the first poll.
        """
        return self._spindle_live_speed

    def get_spindle_rpm(self) -> Optional[int]:
        """Read configured spindle target RPM from device (Pattern B, GET 0x3900).

        RE: get_uint32_0x3900 @ 0x41b220 — dev_trigger_read_uint32s(0x3900, buf, 1).
        Returns RPM as uint32 (big-endian from device), or None on error.
        """
        try:
            data = _usb.trigger_read_b(self._dev, 0x3900, 4)
            if data is None or len(data) < 4:
                log.warning("get_spindle_rpm: short/no response")
                return None
            rpm = struct.unpack_from('>I', bytes(data))[0]
            log.debug("Spindle target RPM read: %d", rpm)
            return rpm
        except usb.core.USBError as e:
            log.warning("get_spindle_rpm failed: %s", e)
            return None

    def set_target_rpm(self, rpm: int) -> int:
        """Update the cached spindle target RPM only — does not touch the device.

        Use while the spindle is off so adjustments to the target accumulate in
        software; the value is pushed to the firmware later by `spindle_on_rpm`
        (typically called from the spindle-toggle handler when the user starts
        the spindle). Returns the clamped RPM that was stored.
        """
        rpm = max(SPINDLE_RPM_MIN, min(SPINDLE_RPM_MAX, int(rpm)))
        self._spindle_target_rpm = rpm
        log.debug("Spindle target RPM cached at %d (not pushed)", rpm)
        return rpm

    def spindle_on_rpm(self, rpm: int) -> None:
        """Set spindle target RPM (SET 0x3901, 1 × big-endian uint32).

        RE: FUN_0041b230 @ 0x41b230 — send_trigger_u32_array(0x3901, &rpm, 1)
        then wait_ping_bit21_clear. RPM is clamped to [4500, 15000].
        Updates the cached _spindle_target_rpm immediately so rapid key presses
        accumulate correctly even before the USB round-trip completes.
        """
        rpm = max(SPINDLE_RPM_MIN, min(SPINDLE_RPM_MAX, int(rpm)))
        self._spindle_target_rpm = rpm   # optimistic update before USB
        try:
            _usb.vend_set(self._dev, 0x3006, struct.pack('>I', rpm))
            log.info("Spindle spindle on RPM %d", rpm)
            self._wait_ping_bit21()
        except usb.core.USBError as e:
            log.warning("set_spindle_rpm failed: %s", e)

    def spindle_off(self) -> None:
        """Stop spindle motor off."""
        try:
            _usb.vend_set(self._dev, 0x3006, struct.pack('>I', 0))
            log.info("Spindle off")
        except usb.core.USBError as e:
            log.warning("spindle_off failed: %s", e)

    def rotary_drill_mode(self, enabled: bool) -> None:
        """A-axis continuous low-speed rotation for center drilling with tailstock.

        RE: [Drill Workpiece] dialog Rotate/Stop buttons (0xffe/0xfff) → FUN_004027a0 /
        FUN_00402840 → FUN_0041b1d0 → SET 0x3809 [1, 0xFFFF] (rotate) / [0, 0] (stop).
        The A-axis spins continuously at low speed; the operator inserts a centre bit
        into the tailstock to bore a center hole for tailstock support.
        """
        payload = struct.pack('<HH', 1, 0xFFFF) if enabled else struct.pack('<HH', 0, 0)
        try:
            _usb.vend_set(self._dev, 0x3809, payload)
            log.info("Rotary drilling %s", "ON" if enabled else "OFF")
        except usb.core.USBError as e:
            log.warning("rotary_drill_mode failed: %s", e)

    def set_spindle_override(self, pct: int) -> None:
        """Update spindle speed while running (10–200 %)."""
        pct = max(10, min(200, int(pct)))
        self._spindle_speed_pct = pct
        try:
            _usb.vend_set(self._dev, 0x3008, bytes([pct]))
            log.info("Spindle speed set to %d%%", pct)
        except usb.core.USBError as e:
            log.warning("set_spindle_speed failed: %s", e)


    # ── Cutting feed rate ─────────────────────────────────────────────────────

    @property
    def cutting_feed_pct(self) -> int:
        return self._cutting_feed_pct

    def set_cutting_feed(self, pct: int) -> None:
        """Set cutting feed rate override % (10–200).

        RE: send_feed_rate_0x307 @ 0041c320.  SET 0x0307 with 1-byte value.
        VPanel only sends this on button press; we send 100% at startup so the
        firmware isn't left at some unknown default.
        """
        pct = max(10, min(200, int(pct)))
        self._cutting_feed_pct = pct
        try:
            _usb.vend_set(self._dev, 0x0307, bytes([pct]))
            log.info("Cutting feed rate set to %d%%", pct)
        except usb.core.USBError as e:
            log.warning("set_cutting_feed failed: %s", e)

    # ── Spindle time ──────────────────────────────────────────────────────────

    def get_spindle_time(self) -> Optional[int]:
        """Read total spindle rotation time from the machine.

        Returns total seconds, or None on error / timeout.
        GET 0x2405 data (58124, 58124, 0, 0, 0)
        GET 0x2405 data (58124, 58124, 0, 0, 0)
        """
        try:
            data = _usb.trigger_read_b(self._dev, 0x2405, 16)
            vals = struct.unpack('>IIHHHxx', data)
            # log.info(f"GET 0x2405 data {vals}")
            seconds = vals[0]
            log.debug("Spindle time: %d s  (%dh %02dm)", seconds, seconds // 3600, (seconds // 60) % 60)
            return seconds
        except usb.core.USBError as e:
            log.warning("get_spindle_time failed: %s", e)
            return None

    @property
    def spindle_secs(self) -> Optional[int]:
        """Total spindle rotation time in seconds, updated by the poll loop (~every 60 s)."""
        return self._spindle_secs

    # ── Machine calibration (Setup → Correction tab) ──────────────────────────
    # See docs/machine-calibration.md for full encoding and wire layout.

    @property
    def axis_scaling_pct(self) -> Optional[tuple]:
        """Per-axis distance scaling (X%, Y%, Z%, A%) from GET 0x5f0.
        Calibration value, read once at handshake. None before that completes.
        """
        return self._axis_scaling_pct

    @property
    def rotary_angle_correction(self) -> Optional[tuple]:
        """Rotary A-axis angle correction (refX1, ofsY1, ofsZ1, refX2, ofsY2, ofsZ2) in mm.
        Calibration value, read once at handshake. None before that completes.
        """
        return self._rotary_angle_correction

    def get_XYZ_axis_scaling(self) -> Optional[tuple]:
        """Read per-axis distance scaling from firmware (Pattern B, GET 0x5f0).

        RE: get_XYZ_axis_scaling_values_0x5f0 @ 0x00419f60. Wire format is
        8 × uint32 BE — 4 numerators followed by 4 denominators, axis order
        X, Y, Z, A. Effective scale = numerator / denominator; multiply by 100
        for the percentage shown in the Setup → Correction dialog.

        Returns (X%, Y%, Z%, A%) tuple, or None on USB / short response.
        """
        try:
            data = _usb.trigger_read_b(self._dev, 0x5f0, 32)
            if data is None or len(data) < 32:
                log.warning("get_XYZ_axis_scaling: short/no response")
                return None
            nums   = struct.unpack_from('>4I', bytes(data), 0)
            denoms = struct.unpack_from('>4I', bytes(data), 16)
            log.debug("GET 0x5f0 raw nums=%s denoms=%s", nums, denoms)
            pct = tuple(
                (n / d) * 100.0 if d else 0.0
                for n, d in zip(nums, denoms)
            )
            self._axis_scaling_pct = pct
            log.info("Axis scaling: X=%.3f%% Y=%.3f%% Z=%.3f%% A=%.3f%%", *pct)
            return pct
        except usb.core.USBError as e:
            log.warning("get_XYZ_axis_scaling failed: %s", e)
            return None

    def get_rotary_axis_angle_correction(self) -> Optional[tuple]:
        """Read rotary A-axis two-point angle correction (Pattern B, GET 0x3804).

        RE: get_rotary_axis_angle_correction_0x3804 @ 0x0041ad10. Wire format
        is 6 × signed int32 BE in 1/1000 mm:
        (refX1, ofsY1, ofsZ1, refX2, ofsY2, ofsZ2). Firmware interpolates the
        Y/Z offsets between the two X reference points.

        Returns the 6-tuple in mm, or None on USB / short response.
        """
        try:
            data = _usb.trigger_read_b(self._dev, 0x3804, 24)
            if data is None or len(data) < 24:
                log.warning("get_rotary_axis_angle_correction: short/no response")
                return None
            vals = struct.unpack_from('>6i', bytes(data))
            log.debug("GET 0x3804 raw %s", vals)
            mm = tuple(v / 1000.0 for v in vals)
            self._rotary_angle_correction = mm
            log.info(
                "Rotary angle correction: P1(X=%.3f Y=%.3f Z=%.3f) P2(X=%.3f Y=%.3f Z=%.3f) mm",
                *mm,
            )
            return mm
        except usb.core.USBError as e:
            log.warning("get_rotary_axis_angle_correction failed: %s", e)
            return None

    ## TODO untested
    def set_XYZ_axis_scaling(
        self,
        x_pct: float, y_pct: float, z_pct: float, a_pct: float,
    ) -> bool:
        """Write per-axis distance scaling (SET 0x5f1).

        RE: set_XYZ_axis_scaling_values_0x5f1 @ 0x0041a110. Payload is 8 × uint32
        BE — 4 numerators followed by 4 denominators. VPanel always sends
        denominators of 1,000,000, so numerator = round(percent × 10000).
        Polls ping bit 21 clear after — returns True on firmware ack.
        """
        SCALE = 10000        # percent × 10000 → numerator with fixed denom 1e6
        DENOM = 1_000_000
        nums = (
            round(x_pct * SCALE),
            round(y_pct * SCALE),
            round(z_pct * SCALE),
            round(a_pct * SCALE),
        )
        payload = struct.pack('>8I', *nums, DENOM, DENOM, DENOM, DENOM)
        try:
            _usb.vend_set(self._dev, 0x5f1, payload)
            log.info("Axis scaling write: X=%.3f%% Y=%.3f%% Z=%.3f%% A=%.3f%%",
                     x_pct, y_pct, z_pct, a_pct)
        except usb.core.USBError as e:
            log.warning("set_XYZ_axis_scaling failed: %s", e)
            return False
        ok = self._wait_ping_bit21()
        if ok:
            self._axis_scaling_pct = (x_pct, y_pct, z_pct, a_pct)
        return ok

    ## TODO untested
    def set_rotary_axis_angle_correction(
        self,
        refX1_mm: float, ofsY1_mm: float, ofsZ1_mm: float,
        refX2_mm: float, ofsY2_mm: float, ofsZ2_mm: float,
    ) -> bool:
        """Write rotary A-axis two-point angle correction (SET 0x3805).

        RE: set_rotary_axis_angle_correction_0x3805 @ 0x0041ad70. Payload is
        6 × signed int32 BE in 1/1000 mm. The firmware expects the lower-X
        point first; this routine swaps the two triples when refX1 > refX2,
        mirroring `settings_write_axis_scaling_and_rotary_offsets`.
        Polls ping bit 21 clear after — returns True on firmware ack.
        """
        if refX1_mm > refX2_mm:
            refX1_mm, refX2_mm = refX2_mm, refX1_mm
            ofsY1_mm, ofsY2_mm = ofsY2_mm, ofsY1_mm
            ofsZ1_mm, ofsZ2_mm = ofsZ2_mm, ofsZ1_mm
        payload = struct.pack(
            '>6i',
            round(refX1_mm * 1000), round(ofsY1_mm * 1000), round(ofsZ1_mm * 1000),
            round(refX2_mm * 1000), round(ofsY2_mm * 1000), round(ofsZ2_mm * 1000),
        )
        try:
            _usb.vend_set(self._dev, 0x3805, payload)
            log.info(
                "Rotary angle correction write: P1(X=%.3f Y=%.3f Z=%.3f) P2(X=%.3f Y=%.3f Z=%.3f) mm",
                refX1_mm, ofsY1_mm, ofsZ1_mm, refX2_mm, ofsY2_mm, ofsZ2_mm,
            )
        except usb.core.USBError as e:
            log.warning("set_rotary_axis_angle_correction failed: %s", e)
            return False
        ok = self._wait_ping_bit21()
        if ok:
            self._rotary_angle_correction = (
                refX1_mm, ofsY1_mm, ofsZ1_mm, refX2_mm, ofsY2_mm, ofsZ2_mm,
            )
        return ok

    def _wait_ping_bit21(self, timeout: float = 3.0) -> bool:
        """Poll GET 0x0001 until bit 21 (_PING_BIT21) clears — firmware command ack.

        RE: wait_ping_bit21_clear (FUN_0041b930) in VP_MDX40A.exe, 3 s timeout.
        Used after SET 0x3901 (spindle RPM), SET 0x2425 (reset spindle time),
        SET 0x3107 (axis config), SET 0x347b-0x3482 (axis params), SET 0x2012 (limits).
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(0.050)
            ping = self._ping_status()
            if ping is None:
                continue
            if ping & _PING_ERROR_MASK:
                log.warning("_wait_ping_bit21: error bit set (0x%08X)", ping)
                return False
            if not (ping & _PING_BIT21):
                return True
        log.warning("_wait_ping_bit21: timeout after %.1f s", timeout)
        return False

    def reset_spindle_time(self, timeout: float = 3.0) -> bool:
        """Reset the spindle rotation time counter to zero.

        RE: setup_dlg_on_reset_spindle / send_reset_spindle_time_0x2425 in VP_MDX40A.exe.
        Sends SET wValue=0x2425, then waits for ping bit 21 (0x200000) to clear.

        Returns True if acknowledged within timeout.
        """
        try:
            _usb.vend_set(self._dev, 0x2425)
            log.info("Spindle time reset sent (SET 0x2425)")
        except usb.core.USBError as e:
            log.warning("reset_spindle_time failed: %s", e)
            return False
        ok = self._wait_ping_bit21(timeout)
        if ok:
            log.debug("Spindle reset acknowledged")
        else:
            log.warning("reset_spindle_time: timeout waiting for ack")
        return ok

    # ── Coordinate systems / WCS ──────────────────────────────────────────────

    @property
    def active_wcs(self) -> int:
        """Active WCS index. 0 = machine coordinates (MCS), 1-10 = WCS1-WCS10."""
        return self._active_wcs

    @property
    def wcs_offset(self) -> tuple:
        """Machine coords of the active WCS origin as (x_mm, y_mm, z_mm, a_deg).
        All zeros when MCS is active. Subtract from GET 0x0100 values for display.
        RE: compute_display_coords @ 0x00403810 — displayed = machine - origin.
        """
        return self._wcs_offset

    def get_wcs_origin(self, slot: int) -> Optional[tuple]:
        """Read stored XYZA origin for WCS slot 1-10 from firmware (Pattern B).

        RE: query_coord_system_by_index @ 0x00403910.
        Returns (x_mm, y_mm, z_mm, a_deg) or None on error.
        """
        if not 1 <= slot <= 10:
            raise ValueError(f"WCS slot must be 1–10, got {slot}")
        wv = _WCS_READ_WVAL[slot - 1]
        try:
            data = _usb.trigger_read_b(self._dev, wv, 16)
            if data is None or len(data) < 16:
                log.warning("get_wcs_origin(%d): short/no response", slot)
                return None
            x, y, z, a = struct.unpack_from('>4i', bytes(data))
            return (x / 1000.0, y / 1000.0, z / 1000.0, a / 1000.0)
        except usb.core.USBError as e:
            log.warning("get_wcs_origin(%d) failed: %s", slot, e)
            return None

    def set_active_wcs(self, slot: int) -> None:
        """Activate WCS slot (0=MCS, 1-10=WCS1-10). Updates the display offset cache.

        After activating, re-reads the new slot's stored origin so the TUI can
        subtract it from GET 0x0100 values, mirroring VPanel's compute_display_coords.
        """
        if not 0 <= slot <= 10:
            raise ValueError(f"WCS slot must be 0–10, got {slot}")
        try:
            log.info("Active WCS → %d", slot)
            self._active_wcs = slot
            if slot == 0:
                self._wcs_offset = (0.0, 0.0, 0.0, 0.0)
            else:
                origin = self.get_wcs_origin(slot)
                self._wcs_offset = origin if origin else (0.0, 0.0, 0.0, 0.0)
        except usb.core.USBError as e:
            log.warning("set_active_wcs(%d) failed: %s", slot, e)

    def write_wcs_origin(
        self, slot: int,
        x_mm: float, y_mm: float, z_mm: float, a_deg: float,
    ) -> bool:
        """Write an explicit XYZA value into a WCS origin slot (1-10).

        RE: MdxAction_x3300+p1_dest (and the WCS1 0x030c variant) — SET wValue
        with 4×uint32 BE, then poll ping bit 21 clear for firmware ack.
        Without the post-write wait, following actions will abort.
        Updates the display offset cache if this slot is currently active.
        Returns True if the firmware acknowledged the write within the
        ping-bit-21 timeout, False on USB error or timeout.
        """
        if not 1 <= slot <= 10:
            raise ValueError(f"WCS slot must be 1–10, got {slot}")
        wv = _WCS_WRITE_WVAL[slot - 1]
        payload = struct.pack(
            '>4i',
            round(x_mm * 1000), round(y_mm * 1000),
            round(z_mm * 1000), round(a_deg * 1000),
        )
        try:
            _usb.vend_set(self._dev, wv, payload)
            log.info("WCS%d origin written: (%.3f, %.3f, %.3f, %.3f°)",
                     slot, x_mm, y_mm, z_mm, a_deg)
            if slot == self._active_wcs:
                self._wcs_offset = (x_mm, y_mm, z_mm, a_deg)
        except usb.core.USBError as e:
            log.warning("write_wcs_origin(%d) failed: %s", slot, e)
            return False
        return self._wait_ping_bit21()

    def partial_update_wcs_origin(self, slot: int, axis: str) -> bool:
        """Overwrite one axis of a stored WCS origin with the live machine position.

        RE: partial_update_coord_sys @ 0x00403a40 — re-reads the slot's stored
        origin, patches the chosen axis from the caller-supplied destination,
        then writes the result back via send_origin_to_coord. The target slot
        does not need to be the active WCS.
        """
        if not 1 <= slot <= 10:
            raise ValueError(f"WCS slot must be 1–10, got {slot}")
        if axis not in ('X', 'Y', 'Z', 'A'):
            raise ValueError(f"axis must be X/Y/Z/A, got {axis!r}")

        origin = self.get_wcs_origin(slot)
        if origin is None:
            log.warning("partial_update_wcs_origin(%d, %s): re-read failed", slot, axis)
            return False

        x, y, z, a = origin
        s = self._state
        if axis == 'X':
            x = s.x_mm
        elif axis == 'Y':
            y = s.y_mm
        elif axis == 'Z':
            z = s.z_mm
        else:
            a = s.a_deg

        return self.write_wcs_origin(slot, x, y, z, a)

    # ── Tool diameter offsets ─────────────────────────────────────────────────

    def get_tool_offsets(self) -> list:
        """Read all 8 tool diameter offset slots from firmware (Pattern B).

        RE: FUN_00402990 — wValue = 0x346a+i for i=1..8, 4-byte uint32 BE.
        Values stored as 1/1000 mm. Returns list of 8 floats (mm), None on error.
        """
        results = []
        for i, wv in enumerate(_TOOL_OFFSET_READ_WVAL, 1):
            try:
                data = _usb.trigger_read_b(self._dev, wv, 4)
                if data is None or len(data) < 4:
                    log.warning("get_tool_offset(%d): short/no response", i)
                    results.append(None)
                else:
                    raw = struct.unpack_from('>I', bytes(data))[0]
                    results.append(raw / 1000.0)
            except usb.core.USBError as e:
                log.warning("get_tool_offset(%d) failed: %s", i, e)
                results.append(None)
        return results

    def set_tool_offset(self, slot: int, value_mm: float) -> bool:
        """Write one tool diameter offset slot (1-8) to firmware.

        RE: apply_axis_config_to_device — wValue = 0x347a+i, 4-byte uint32 BE.
        Polls ping bit 21 clear after write (firmware ack). Returns True if acked.
        """
        if not 1 <= slot <= 8:
            raise ValueError(f"Tool offset slot must be 1–8, got {slot}")
        wv = _TOOL_OFFSET_WRITE_WVAL[slot - 1]
        raw = max(0, round(value_mm * 1000))
        try:
            _usb.vend_set(self._dev, wv, struct.pack('>I', raw))
            log.info("Tool offset T%d → %.3f mm (raw %d)", slot, value_mm, raw)
        except usb.core.USBError as e:
            log.warning("set_tool_offset(%d) failed: %s", slot, e)
            return False
        return self._wait_ping_bit21()

    # ── Rotary A-axis ─────────────────────────────────────────────────────────

    @property
    def rotary_extension_byte(self) -> Optional[int]:
        """Latest GET 0x3800 byte (refreshed on every poll).
        0 = no rotary attachment, 1 = rotary axis only, 2 = rotary + vice headstock.
        None until the first poll completes. See docs/machine-state.md.
        """
        return self._rotary_extension_byte

    @property
    def rotary_centerline(self) -> Optional[tuple]:
        """Stored A-axis rotary centreline (x_mm, y_mm, z_mm) in machine coords.

        Calibration value (set by the Detect Jig routine via SET 0x3803), static
        at runtime. Populated lazily by `poll()` the first time the machine
        reaches idle state — None before that. Only Y, Z define the line; the
        X component is the nominal touch-off X and is informational.
        """
        return self._rotary_centerline

    @property
    def firmware_id(self) -> Optional[str]:
        """ASCII model/firmware identification string read at startup (GET 0x0101).

        None if the handshake hasn't completed or the device didn't respond.
        """
        return self._firmware_id

    def get_firmware_id(self) -> Optional[str]:
        """Query the machine for its ASCII model/firmware string (Pattern B, GET 0x0101).

        RE: AutoMachineClass::get_ascii_str_trigger_0x101 @ 0x41c190 — bare SET 0x0101
        trigger then dev_read_response (ping-poll + GET 0x0003), max 256 bytes,
        result stored as a null-terminated ASCII CString.
        """
        try:
            data = _usb.trigger_read_b(self._dev, 0x0101, 256)
            if data is None or not len(data):
                log.warning("get_firmware_id: no response")
                return None
            # Truncate at first NUL (CString semantics) and decode as ASCII.
            raw = bytes(data)
            nul = raw.find(b'\x00')
            if nul >= 0:
                raw = raw[:nul]
            text = raw.decode('ascii', errors='replace').strip()
            log.debug("Firmware ID: %r", text)
            return text
        except (usb.core.USBError, ValueError) as e:
            log.warning("get_firmware_id failed: %s", e)
            return None

    def get_rotary_axis_centreline(self) -> Optional[tuple]:
        """Read stored rotary A-axis centreline from firmware (Pattern B, GET 0x3801).

        RE: get_rotary_axis_centreline_0x3801 — dev_trigger_read_uint32s(0x3801, buf, 3).
        Returns (x_mm, y_mm, z_mm) signed int32 BE in 1/1000 mm, or None on error.
        """
        try:
            data = _usb.trigger_read_b(self._dev, 0x3801, 12)
            if data is None or len(data) < 12:
                log.warning("get_rotary_axis_centreline: short/no response")
                return None
            x, y, z = struct.unpack_from('>3i', bytes(data))
            return (x / 1000.0, y / 1000.0, z / 1000.0)
        except usb.core.USBError as e:
            log.warning("get_rotary_axis_centreline failed: %s", e)
            return None

    def move_to_machine_pos(
        self,
        x_mm: float, y_mm: float, z_mm: float, a_deg: float,
        speed: int = 1800,
    ) -> None:
        """Absolute move to machine-coordinate target (SET 0x04f7).

        RE: FUN_00419ef0 @ 0x00419ef0 — absolute position move (flags=0xFFFF),
        wrapped in operation bracket SET 0x1109 (0x00 begin / 0xff end).
        Fire-and-forget: returns once the four control transfers are sent;
        the caller observes motion completion via subsequent poll() cycles.
        """
        payload = struct.pack(
            '>HH4i', speed, 0xFFFF,
            round(x_mm * 1000), round(y_mm * 1000),
            round(z_mm * 1000), round(a_deg * 1000),
        )
        try:
            _usb.vend_set(self._dev, 0x1109, b'\x00')
            _usb.vend_set(self._dev, 0x04f7, payload)
            _usb.vend_set(self._dev, 0x3f2)
            _usb.vend_set(self._dev, 0x1109, b'\xff')
            log.info("Move to machine (%.3f, %.3f, %.3f, %.3f°) speed=%d",
                     x_mm, y_mm, z_mm, a_deg, speed)
        except usb.core.USBError as e:
            log.warning("move_to_machine_pos failed: %s", e)

    # ── Move-to-origin presets (firmware-resolved) ────────────────────────────

    _VALID_MOVE_AXIS_MASKS = (1, 2, 3, 4, 8)   # X, Y, XY, Z, A — see docs/usb-protocol.md

    def _send_bracketed(self, wValue: int, payload: bytes, log_msg: str) -> None:
        """Send a single SET wrapped in the 0x1109 operation bracket."""
        try:
            _usb.vend_set(self._dev, 0x1109, b'\x00')
            _usb.vend_set(self._dev, wValue, payload)
            _usb.vend_set(self._dev, 0x1109, b'\xff')
            log.info(log_msg)
        except usb.core.USBError as e:
            log.warning("SET 0x%04x failed: %s", wValue, e)

    def move_to_view_position(self, speed: int = JOG_SPEED_MAX) -> None:
        """Move to the parked front-of-bed View Position (SET 0x0500).

        RE: move_to_view_position_x0500 @ 0x41c3f0 — send_trigger_u16_array(0x500, &speed, 1).
        Used by the main panel's Move button (target = "View Position").
        """
        self._send_bracketed(0x0500, struct.pack('>H', speed),
                             f"Move to View Position (speed={speed})")

    def move_to_origin(
        self,
        axis_mask: int,
        wcs: Optional[int] = None,
        speed: int = JOG_SPEED_MAX,
    ) -> None:
        """Move the selected axes to their stored origin in the given WCS (SET 0x3501).

        Payload `>HHH` — (wcs_code, axis_mask, speed). `wcs` defaults to the
        currently-active WCS. `axis_mask` is a bitfield: 1=X, 2=Y, 4=Z, 8=A
        (and combinations: 3=XY). VPanel only ever sends {1, 2, 3, 4, 8} from
        the Move dropdown — anything else is rejected.

        RE: MdxAction_x3501_MegaDispatch @ 0x403eb0 leaves.
        See docs/usb-protocol.md "Move-to-origin payload" for full encoding.
        """
        if axis_mask not in self._VALID_MOVE_AXIS_MASKS:
            raise ValueError(
                f"axis_mask must be one of {self._VALID_MOVE_AXIS_MASKS}, got {axis_mask}"
            )
        wcs_code = self._active_wcs if wcs is None else wcs
        self._send_bracketed(
            0x3501, struct.pack('>HHH', wcs_code, axis_mask, speed),
            f"Move to origin: wcs={wcs_code} mask=0x{axis_mask:x} speed={speed}",
        )

    def move_to_rotation_center_y(self, speed: int = JOG_SPEED_MAX) -> None:
        """Move Y onto the rotary A-axis centreline (SET 0x3808, mode=2).

        Payload `>HH` — (mode=2, speed). Rotary-attachment-only function;
        firmware uses the stored centreline (see get_rotary_axis_centreline)
        to compute the Y target. Z is untouched.

        RE: move_to_rotation_centre_x3808 @ 0x41b1a0 —
        send_trigger_u16_array(0x3808, &[2, speed], 2).
        """
        self._send_bracketed(0x3808, struct.pack('>HH', 2, speed),
                             f"Move Y to rotation centre (speed={speed})")

    # ── Internal ─────────────────────────────────────────────────────────────

    _SPINDLE_POLL_INTERVAL = 60.0  # seconds between get_spindle_time() refreshes (wall-clock, not call count)

    def _handshake(self) -> None:
        """Perform the VPanel startup sequence."""
        dev = self._dev

        # USB printer class GET_DEVICE_ID (bmRequestType=0xA1, bRequest=0x00).
        # On Windows, USBPRINT.SYS sends this automatically during device
        # enumeration — before VPanel ever opens the handle. On macOS with
        # libusb it is never sent. The machine firmware may require it to
        # transition from standalone mode to remote-control mode.
        try:
            data = dev.ctrl_transfer(0xA1, 0x00, 0x0000, self._intf_num, 1024, timeout=2000)
            log.info("GET_DEVICE_ID (%d bytes): %s",
                     len(data), bytes(data[:64]).decode('ascii', errors='replace'))
        except usb.core.USBError as e:
            log.debug("GET_DEVICE_ID not supported or failed: %s", e)

        # USB printer class SOFT_RESET (bmRequestType=0x21, bRequest=0x02).
        # USBPRINT may send this when first opening the port. Harmless if unsupported.
        try:
            dev.ctrl_transfer(0x21, 0x02, 0x0000, self._intf_num, 0, timeout=2000)
            log.info("SOFT_RESET sent")
        except usb.core.USBError as e:
            log.debug("SOFT_RESET not supported or failed: %s", e)

        # Ping
        self._ping_status()

        # Endian detection, 4 byte read should be 0x1234
        try:
            data = _usb.vend_get(dev, 0x0002, 4)
            _,_,sig = struct.unpack(">BBH", data)
            if sig == 0x1234:
                log.info("Machine endian confirmed (sig=0x1234)")
            else:
                print("Unexpected machine endian sig=0x%04X (want 0x1234)", sig)
                exit(-1)
        except usb.core.USBError as e:
            log.warning("Machine type check failed: %s", e)

        # Keepalive — VPanel sends 1 byte payload (poll_keepalive @ 0041c3d0)
        try:
            _usb.vend_set(dev, 0x03f5, b'\x00')
            log.debug("Keepalive sent")
        except usb.core.USBError as e:
            log.warning("Keepalive failed: %s", e)

        # Read ASCII model/firmware identification string (GET 0x0101).
        # RE: AutoMachineClass::get_ascii_str_trigger_0x101 @ 0x41c190.
        fw = self.get_firmware_id()
        if fw:
            self._firmware_id = fw
            log.info("Firmware ID: %s", fw)

        # Read calibration (Setup → Correction tab values): per-axis scaling
        # and rotary two-point angle correction. Static at runtime — cached
        # on self for the lifetime of the connection.
        self.get_XYZ_axis_scaling()
        self.get_rotary_axis_angle_correction()

        # Read configured spindle target RPM (GET 0x3900, Pattern B)
        rpm = self.get_spindle_rpm()
        if rpm is not None and SPINDLE_RPM_MIN <= rpm <= SPINDLE_RPM_MAX:
            self._spindle_target_rpm = rpm
            log.info("Spindle target RPM read from device: %d", rpm)
        else:
            log.info("Spindle RPM read failed or out of range (%s); defaulting to %d", rpm, SPINDLE_RPM_MIN)

    def _read_state(self) -> Optional[MachineState]:
        """Read the 0x0100 state block (XYZA coords + flags) and cache on self._state.

        Sequence mirrors VPanel's 200 ms poll (FUN_00403710): SET 0x03f5
        keepalive, then Pattern B trigger reads for 0x3005/0x3800/0x3003/0x3b01
        (SET wValue → GET 0x0003 — we issue the GET immediately because
        ping[3] never goes non-zero on macOS/libusb, but the firmware still
        needs the complete SET→GET cycle). XYZA state (GET 0x0100) is a
        Pattern A direct read at the end.

        Distinct from _ping_status(), which reads the separate 4-byte ping
        register at 0x0001 (motion-busy / command-ack / response-length).
        """
        try:
            _usb.vend_set(self._dev, 0x03f5, b'\x00')
            for wv, n in ((0x3005, 8), (0x3800, 1), (0x3003, 4), (0x3b01, 4)):
                try:
                    raw = _usb.trigger_read(self._dev, wv, n)
                    if raw is not None and len(raw):
                        if wv == 0x3800:
                            self._rotary_extension_byte = raw[0]
                        elif wv == 0x3005 and len(raw) >= 8:
                            self._spindle_range = struct.unpack_from('>2I', bytes(raw))
                            log.debug(f"GET 0x{wv:0x} spindle range {self._spindle_range}")
                        elif wv == 0x3003 and len(raw) >= 4:
                            self._spindle_live_speed = struct.unpack_from('>I', bytes(raw))[0]
                            log.debug(f"GET 0x{wv:0x} spindle live speed {self._spindle_live_speed}")
                        else:
                            log.debug(f"GET 0x{wv:0x} returned: {raw}")
                except usb.core.USBError as e:
                    log.debug("Poll read 0x%04x failed: %s", wv, e)

            data = _usb.vend_get(self._dev, 0x0100, 32)
            if len(data) < 20:
                log.debug("Short state response (%d bytes)", len(data))
                return None
            s = _decode_state(data)
            self._state = s
            return s
        except usb.core.USBTimeoutError:
            log.debug("State read timeout")
            return None
        except usb.core.USBError as e:
            log.debug("State read error: %s", e)
            return None

    # ── NC / RML file output ──────────────────────────────────────────────────

    def begin_nc_job(self) -> None:
        """Open NC operation bracket (SET 0x1109 = 0x00) before bulk data.

        RE: execute_cut_job @ 0x00416360 calls send_operation_bracket_0x1109
        with 0x00 before starting the NC job. Without this the firmware ignores
        bulk NC data. Pair with end_nc_job() when all data has been sent.
        """
        self._set_operation_mode(True)

    def end_nc_job(self) -> None:
        """Close NC operation bracket (SET 0x1109 = 0xFF) after bulk data."""
        self._set_operation_mode(False)

    def bulk_write(self, data: bytes) -> int:
        """Write raw bytes to the bulk-OUT endpoint (NC/RML command stream)."""
        return _usb.bulk_write(self._dev, data)

    def get_nc_bytes_processed(self) -> int:
        """Read NC bytes-processed counter (Pattern B, wValue=0x0200).

        RE: get_coord_pair_0x200 @ 0x0041c290 — SET 0x0200 → GET 0x0003, 4 bytes BE.
        Firmware increments this as it consumes NC data from its internal buffer.
        Returns unsigned 32-bit counter, or -1 on error.
        """
        try:
            data = _usb.trigger_read(self._dev, 0x0200, 4)
            if data and len(data) >= 4:
                return struct.unpack('>I', bytes(data[:4]))[0]
        except usb.core.USBError as e:
            log.debug("get_nc_bytes_processed failed: %s", e)
        return -1
