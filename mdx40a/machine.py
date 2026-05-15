"""
Roland MDX-40A machine interface.

MDX40A manages a background polling thread (200 ms) that reads machine state
and notifies registered observers via callbacks.  All USB interaction goes
through mdx40a.usb primitives — no direct libusb calls here.
"""

import logging
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import usb.core

from . import usb as _usb

log = logging.getLogger(__name__)

POLL_INTERVAL = 0.200   # seconds

# Jog speeds (mm/min). VPanel sends 0xFFFF (max) for single-press steps;
# for a meaningful slow/fast difference we use VPanel's continuous-ramp bounds.
# RE: DAT_00440aa0=240 (ramp start), DAT_00440aac=8000 (XYZ ramp max).
JOG_SPEED_SLOW =   240   # VPanel continuous jog ramp start (XYZ)
JOG_SPEED_FAST = 0xFFFF  # VPanel single-press speed (firmware maximum)

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
#  bit 13  0x00002000  BUSY       motion in progress (wait_motion_complete_loop exit)
#  bit 12  0x00001000  ERROR      error condition (loop returns failure)
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
        machine.on_state(lambda s: print(s))
        machine.jog('Z', -2.0)   # move 2 mm down
        machine.release()
    """

    def __init__(self):
        self._dev = None
        self._intf_num: Optional[int] = None
        self._state = MachineState()
        self._state_lock = threading.Lock()
        self._usb_lock = threading.Lock()   # serialises all USB control transfers
        self._observers: List[Callable[[MachineState], None]] = []
        self._poll_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._spindle_speed_pct: int = 100   # cached spindle override %
        self._cutting_feed_pct: int = 100    # cached cutting feed override %
        self._spindle_target_rpm: int = SPINDLE_RPM_MIN  # configured target RPM (GET/SET 0x3900/0x3901)
        self._spindle_secs: Optional[int] = None
        self._active_wcs: int   = 0                      # 0=MCS, 1-10=WCS1-WCS10
        self._wcs_offset: tuple = (0.0, 0.0, 0.0, 0.0)  # machine coords of active WCS origin (mm/deg)

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
        self._stop_event.clear()
        self._poll_thread = threading.Thread(
            target=self._poll_loop, name="mdx40a-poll", daemon=True
        )
        self._poll_thread.start()
        log.info("Polling thread started")

    def release(self) -> None:
        """Stop the polling thread and release the USB interface."""
        self._stop_event.set()
        if self._poll_thread:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None
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
        with self._state_lock:
            return self._state

    def on_state(self, callback: Callable[[MachineState], None]) -> None:
        """Register a callback invoked on every successful poll."""
        self._observers.append(callback)

    # ── Jog ──────────────────────────────────────────────────────────────────

    def jog(
        self,
        axis: str,
        dist_mm: float,
        speed: int = JOG_SPEED_SLOW,
        timeout: float = 30.0,
    ) -> MachineState:
        """
        Move `axis` by `dist_mm` relative to current machine position.

        Blocks until the axis settles or `timeout` seconds elapse.
        axis: 'X', 'Y', 'Z', or 'A' (A in degrees)
        """
        axis = axis.upper()
        if axis not in ('X', 'Y', 'Z', 'A'):
            raise ValueError(f"axis must be X/Y/Z/A, got {axis!r}")

        with self._state_lock:
            s = self._state
        if not s.ready:
            raise RuntimeError("Machine state not yet available; connect first")

        # Build relative displacement vector (1/1000 mm units).
        # wValue=0x4f5 takes a displacement, not an absolute target.
        # Only the jogged axis is non-zero; others are 0.
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
        return self._wait_settle(axis, timeout)

    def _set_operation_mode(self, active: bool) -> None:
        """SET 0x1109 operation bracket (RE: FUN_0041c410).

        VPanel sends 0x00 before any interactive operation (jog, detect jig, move-to)
        and 0xff after. Hypothesis: releases the firmware parking brake / enables
        servo drive for remote commands.
        """
        payload = b'\x00' if active else b'\xff'
        try:
            with self._usb_lock:
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
        with self._usb_lock:
            _usb.vend_set(self._dev, 0x04f5, payload)
        log.debug("Jog cmd sent: %s", payload.hex())

    def _ping_status(self) -> int:
        """Read 4-byte ping status word (GET wValue=0x0001). Returns -1 on error."""
        try:
            with self._usb_lock:
                data = _usb.vend_get(self._dev, 0x0001, 4)
            if len(data) < 4:
                return -1
            return struct.unpack_from('<I', data)[0]
        except usb.core.USBError:
            return -1

    def _wait_motion_complete(self, timeout: float) -> bool:
        """Wait for jog motion to complete (RE: jog_wait_busy_bits_clear @ 0x00417b00).

        Polls ping every 100 ms until bits 2 and 21 (0x00200004) are clear for
        two consecutive readings, then sends the motion-done ack.

        Fast moves that finish before the first poll are handled correctly: BUSY
        will already be clear, so two consecutive clear readings happen immediately.
        """
        deadline = time.monotonic() + timeout
        last_clear = False
        while time.monotonic() < deadline:
            time.sleep(0.100)
            ping = self._ping_status()
            if ping == -1:
                log.warning("Ping read failed during motion wait")
                last_clear = False
                continue
            log.debug("Motion wait ping: 0x%08X  busy=%s", ping, bool(ping & _PING_BUSY_MASK))
            if ping & _PING_ERROR_MASK:
                log.warning("Ping error bit in motion wait (0x%08X)", ping)
                return False
            now_clear = (ping & _PING_BUSY_MASK) == 0
            if now_clear and last_clear:
                log.debug("Motion complete")
                return True
            last_clear = now_clear
        log.warning("Motion wait timed out after %.1f s", timeout)
        return False

    def stop_motion(self) -> None:
        """Send immediate motion stop (SET wValue=0x03f3)."""
        try:
            with self._usb_lock:
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

    def get_spindle_rpm(self) -> Optional[int]:
        """Read configured spindle target RPM from device (Pattern B, GET 0x3900).

        RE: get_uint32_0x3900 @ 0x41b220 — dev_trigger_read_uint32s(0x3900, buf, 1).
        Returns RPM as uint32 (big-endian from device), or None on error.
        """
        try:
            with self._usb_lock:
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

    def set_spindle_rpm(self, rpm: int) -> None:
        """Set spindle target RPM (SET 0x3901, 1 × big-endian uint32).

        RE: FUN_0041b230 @ 0x41b230 — send_trigger_u32_array(0x3901, &rpm, 1)
        then wait_ping_bit21_clear. RPM is clamped to [4500, 15000].
        Updates the cached _spindle_target_rpm immediately so rapid key presses
        accumulate correctly even before the USB round-trip completes.
        """
        rpm = max(SPINDLE_RPM_MIN, min(SPINDLE_RPM_MAX, int(rpm)))
        self._spindle_target_rpm = rpm   # optimistic update before USB
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x3901, struct.pack('>I', rpm))
            log.info("Spindle target RPM → %d", rpm)
            self._wait_ping_bit21()
        except usb.core.USBError as e:
            log.warning("set_spindle_rpm failed: %s", e)

    def spindle_on(self) -> None:
        """Start spindle motor (RE: main panel Start/Stop Spindle button → SET 0x03f0 bare trigger)."""
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x03f0)
            log.info("Spindle on (SET 0x03f0)")
        except usb.core.USBError as e:
            log.warning("spindle_on failed: %s", e)

    def spindle_off(self) -> None:
        """Stop spindle motor (RE: main panel Start/Stop Spindle button → SET 0x03f1 bare trigger)."""
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x03f1)
            log.info("Spindle off (SET 0x03f1)")
        except usb.core.USBError as e:
            log.warning("spindle_off failed: %s", e)

    def rotary_drill_mode(self, enabled: bool) -> None:
        """A-axis continuous low-speed rotation for center drilling with tailstock.

        RE: [Drill Workpiece] dialog Rotate/Stop buttons (0xffe/0xfff) → FUN_004027a0 /
        FUN_00402840 → FUN_0041b1d0 → SET 0x3809 [1, 0xFFFF] (rotate) / [0, 0] (stop).
        The A-axis spins continuously at low speed; the operator uses a hand-held drill
        to bore a center hole for tailstock support.
        """
        payload = struct.pack('<HH', 1, 0xFFFF) if enabled else struct.pack('<HH', 0, 0)
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x3809, payload)
            log.info("Rotary drill mode %s (SET 0x3809 %s)",
                     "ON" if enabled else "OFF",
                     "[1, 0xFFFF]" if enabled else "[0, 0]")
        except usb.core.USBError as e:
            log.warning("rotary_drill_mode failed: %s", e)

    def set_spindle_speed(self, pct: int) -> None:
        """Update spindle speed while running (10–200 %)."""
        pct = max(10, min(200, int(pct)))
        self._spindle_speed_pct = pct
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x3008, bytes([pct]))
            log.info("Spindle speed set to %d%%", pct)
        except usb.core.USBError as e:
            log.warning("set_spindle_speed failed: %s", e)

    def set_spindle_speed_cached(self, pct: int) -> None:
        """Update the cached spindle speed without sending (spindle is off)."""
        self._spindle_speed_pct = max(10, min(200, int(pct)))

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
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x0307, bytes([pct]))
            log.info("Cutting feed rate set to %d%%", pct)
        except usb.core.USBError as e:
            log.warning("set_cutting_feed failed: %s", e)

    # ── Spindle time ──────────────────────────────────────────────────────────

    def get_spindle_time(self) -> Optional[int]:
        """Read total spindle rotation time from the machine.

        RE: setup_dlg_show_spindle_time / get_status_struct_0x2405 in VP_MDX40A.exe.
        Mirrors dev_trigger_read → dev_send_trigger + FUN_0041b990 + dev_read_response:
          1. SET 0x2405  — prime the device
          2. Poll GET 0x0001 until ping byte[3] (C's LE high byte) goes non-zero;
             that byte IS the response length (FUN_0041b990, 3 s timeout).
          3. GET 0x0003 of that many bytes — first uint32 (big-endian) = seconds.

        Returns total seconds, or None on error / timeout.
        """
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x2405)
                deadline = time.monotonic() + 3.0
                length = 0
                while time.monotonic() < deadline:
                    ping = _usb.vend_get(self._dev, 0x0001, 4)
                    if len(ping) >= 4:
                        log.debug("get_spindle_time: ping %s", bytes(ping).hex())
                        if ping[2] & 0x10:   # bit 20 = device error (bit 4 of byte[2])
                            log.warning("get_spindle_time: error bit in ping %s",
                                        bytes(ping).hex())
                            return None
                        length = ping[3]     # C LE high byte = response length
                        if length:
                            break
                    time.sleep(0.020)
                if not length:
                    log.warning("get_spindle_time: timeout waiting for device response")
                    return None
                data = _usb.vend_get(self._dev, 0x0003, min(length, 16))
            if len(data) < 4:
                log.warning("get_spindle_time: short response (%d bytes)", len(data))
                return None
            seconds = struct.unpack_from('>I', data, 0)[0]
            log.debug("Spindle time: %d s  (%dh %02dm)", seconds, seconds // 3600, (seconds // 60) % 60)
            return seconds
        except usb.core.USBError as e:
            log.warning("get_spindle_time failed: %s", e)
            return None

    @property
    def spindle_secs(self) -> Optional[int]:
        """Total spindle rotation time in seconds, updated by the poll loop (~every 60 s)."""
        return self._spindle_secs

    def get_device_status_0x3804(self) -> Optional[tuple]:
        """Read the 6-uint32 device status block (RE: get_6uint32_0x3804 @ 0x0041ad10).

        Pattern B trigger read: SET 0x3804 → poll ping[3] → GET 0x0003, 24 bytes BE.
        VPanel stores the result at this+0x8c..0xa0 and uses it in two ways:
          - FUN_00403e90: word[0] bit 2 (0x4) gates jog commands (suppresses if set)
          - FUN_004042e0: same bit is the motion-complete wait exit condition

        Returns tuple of 6 LE uint32s, or None on error/timeout.
        """
        try:
            with self._usb_lock:
                data = _usb.trigger_read_b(self._dev, 0x3804, 24)
            if data is None or len(data) < 24:
                log.warning("get_device_status_0x3804: short/no response (%s)",
                            None if data is None else len(data))
                return None
            vals = struct.unpack_from('>6I', bytes(data))
            busy = bool(vals[0] & 0x04)
            log.info(
                "GET 0x3804: [%s]  word0=0x%08x  busy(bit2)=%s",
                ' '.join(f'{v:08x}' for v in vals), vals[0], busy,
            )
            return vals
        except usb.core.USBError as e:
            log.warning("get_device_status_0x3804 failed: %s", e)
            return None

    def _wait_ping_bit21(self, timeout: float = 3.0) -> bool:
        """Poll GET 0x0001 until bit 21 (_PING_BIT21) clears — firmware command ack.

        RE: wait_ping_bit21_clear (FUN_0041b930) in VP_MDX40A.exe, 3 s timeout.
        Used after SET 0x3901 (spindle RPM), SET 0x2425 (reset spindle time),
        SET 0x3107 (axis config), SET 0x347b-0x3482 (axis params), SET 0x2012 (limits).
        Must NOT be called while holding _usb_lock; _ping_status() acquires it internally.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(0.050)
            ping = self._ping_status()
            if ping == -1:
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
            with self._usb_lock:
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
            with self._usb_lock:
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

        RE: set_coordinate_system @ 0x0041a300 — SET 0x3006, 1×uint32 index.
        After activating, re-reads the new slot's stored origin so the TUI can
        subtract it from GET 0x0100 values, mirroring VPanel's compute_display_coords.
        """
        if not 0 <= slot <= 10:
            raise ValueError(f"WCS slot must be 0–10, got {slot}")
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x3006, struct.pack('>I', slot))
            log.info("Active WCS → %d", slot)
            self._active_wcs = slot
            if slot == 0:
                self._wcs_offset = (0.0, 0.0, 0.0, 0.0)
            else:
                origin = self.get_wcs_origin(slot)
                self._wcs_offset = origin if origin else (0.0, 0.0, 0.0, 0.0)
        except usb.core.USBError as e:
            log.warning("set_active_wcs(%d) failed: %s", slot, e)

    def capture_origin(self, slot: Optional[int] = None) -> bool:
        """Latch current machine position as the WCS origin (SET 0x3f2).

        If `slot` differs from the active WCS, activates it first (SET 0x3006).
        RE: on_cmd_set_origin_point @ 0x00416AB0 — bare trigger SET 0x3f2; firmware
        captures its encoder positions into the currently active WCS slot.
        """
        target = slot if slot is not None else self._active_wcs
        if target == 0:
            log.warning("capture_origin: cannot capture into MCS (slot 0)")
            return False
        try:
            if target != self._active_wcs:
                with self._usb_lock:
                    _usb.vend_set(self._dev, 0x3006, struct.pack('>I', target))
                self._active_wcs = target
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x3f2)
            log.info("Origin captured into WCS%d", target)
            origin = self.get_wcs_origin(target)
            if origin:
                self._wcs_offset = origin
            return True
        except usb.core.USBError as e:
            log.warning("capture_origin failed: %s", e)
            return False

    def write_wcs_origin(
        self, slot: int,
        x_mm: float, y_mm: float, z_mm: float, a_deg: float,
    ) -> None:
        """Write an explicit XYZA value into a WCS origin slot (1-10).

        RE: FUN_00403a40 — SET 0x030c (WCS1) / 0x3335-0x333d (WCS2-10), 4×uint32 BE.
        Updates the display offset cache if this slot is currently active.
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
            with self._usb_lock:
                _usb.vend_set(self._dev, wv, payload)
            log.info("WCS%d origin written: (%.3f, %.3f, %.3f, %.3f°)",
                     slot, x_mm, y_mm, z_mm, a_deg)
            if slot == self._active_wcs:
                self._wcs_offset = (x_mm, y_mm, z_mm, a_deg)
        except usb.core.USBError as e:
            log.warning("write_wcs_origin(%d) failed: %s", slot, e)

    def move_to_machine_pos(
        self,
        x_mm: float, y_mm: float, z_mm: float, a_deg: float,
        speed: int = 1800,
    ) -> None:
        """Absolute move to machine-coordinate target (SET 0x04f7).

        RE: FUN_00419ef0 @ 0x00419ef0 — absolute position move (flags=0xFFFF),
        wrapped in operation bracket SET 0x1109 (0x00 begin / 0xff end).
        The TUI passes machine coords here; callers convert from WCS if needed.
        """
        payload = struct.pack(
            '>HH4i', speed, 0xFFFF,
            round(x_mm * 1000), round(y_mm * 1000),
            round(z_mm * 1000), round(a_deg * 1000),
        )
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x1109, b'\x00')
                _usb.vend_set(self._dev, 0x04f7, payload)
                _usb.vend_set(self._dev, 0x3f2)
                _usb.vend_set(self._dev, 0x1109, b'\xff')
            log.info("Move to machine (%.3f, %.3f, %.3f, %.3f°) speed=%d",
                     x_mm, y_mm, z_mm, a_deg, speed)
        except usb.core.USBError as e:
            log.warning("move_to_machine_pos failed: %s", e)

    def _wait_settle(self, axis: str, timeout: float) -> MachineState:
        """Wait for motion to complete using firmware ping bits, then read final position."""
        completed = self._wait_motion_complete(timeout)
        s = self.state
        pos = {'X': s.x_mm, 'Y': s.y_mm, 'Z': s.z_mm, 'A': s.a_deg}[axis]
        if completed:
            log.info("Jog complete: %s=%.3f mm", axis, pos)
        else:
            log.warning("Jog %s timed out; position is %.3f mm", axis, pos)
        return s

    # ── Internal ─────────────────────────────────────────────────────────────

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
        try:
            with self._usb_lock:
                data = _usb.vend_get(dev, 0x0001, 4)
            log.debug("Ping (GET 0x0001): %s", bytes(data).hex())
        except usb.core.USBError as e:
            log.warning("Ping failed: %s", e)

        # Machine type
        try:
            with self._usb_lock:
                data = _usb.vend_get(dev, 0x0002, 4)
            sig = (data[2] << 8) | data[3]
            if sig == 0x1234:
                log.info("Machine type confirmed (sig=0x1234)")
            else:
                log.warning("Unexpected machine type sig=0x%04X (want 0x1234)", sig)
        except usb.core.USBError as e:
            log.warning("Machine type check failed: %s", e)

        # Keepalive — VPanel sends 1 byte payload (poll_keepalive @ 0041c3d0)
        try:
            with self._usb_lock:
                _usb.vend_set(dev, 0x03f5, b'\x00')
            log.debug("Keepalive sent")
        except usb.core.USBError as e:
            log.warning("Keepalive failed: %s", e)

        # Read initial 0x3804 device status (6×uint32 busy/status block)
        self.get_device_status_0x3804()

        # Read configured spindle target RPM (GET 0x3900, Pattern B)
        rpm = self.get_spindle_rpm()
        if rpm is not None and SPINDLE_RPM_MIN <= rpm <= SPINDLE_RPM_MAX:
            self._spindle_target_rpm = rpm
            log.info("Spindle target RPM read from device: %d", rpm)
        else:
            log.info("Spindle RPM read failed or out of range (%s); defaulting to %d", rpm, SPINDLE_RPM_MIN)

    def _read_state(self) -> Optional[MachineState]:
        """Read machine state, mirroring VPanel's 200ms poll then XYZA read.

        VPanel poll (FUN_00403710): SET 0x03f5 (keepalive, 1-byte payload) then Pattern B
        trigger reads for 0x3005/0x3800/0x3003/0x3b01 — each is SET wValue → GET 0x0003.
        We use trigger_read (immediate GET, no ping-poll wait) because ping[3] never goes
        non-zero on macOS/libusb. The machine still needs the complete SET→GET cycle.
        XYZA state (GET 0x0100) is a separate Pattern A direct read after the poll.
        """
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x03f5, b'\x00')
                for wv, n in ((0x3005, 8), (0x3800, 1), (0x3003, 4), (0x3b01, 4)):
                    try:
                        raw = _usb.trigger_read(self._dev, wv, n)
                        if raw is not None and wv == 0x3800 and len(raw):
                            log.debug("GET 0x3800 status byte: 0x%02x", raw[0])
                    except usb.core.USBError as e:
                        log.debug("Poll read 0x%04x failed: %s", wv, e)
                data = _usb.vend_get(self._dev, 0x0100, 32)
            if len(data) < 20:
                log.debug("Short state response (%d bytes)", len(data))
                return None
            return _decode_state(data)
        except usb.core.USBTimeoutError:
            log.debug("State read timeout")
            return None
        except usb.core.USBError as e:
            log.debug("State read error: %s", e)
            return None

    _SPINDLE_POLL_EVERY = round(60.0 / POLL_INTERVAL)  # ticks between spindle-time reads

    def _poll_loop(self) -> None:
        spindle_tick = 0
        while not self._stop_event.is_set():
            s = self._read_state()
            if s is not None:
                with self._state_lock:
                    self._state = s
                for cb in self._observers:
                    try:
                        cb(s)
                    except Exception:
                        log.exception("Observer callback raised")
            spindle_tick += 1
            if spindle_tick >= self._SPINDLE_POLL_EVERY:
                spindle_tick = 0
                secs = self.get_spindle_time()
                if secs is not None:
                    self._spindle_secs = secs
            self._stop_event.wait(POLL_INTERVAL)
