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

# Ping status bits (GET wValue=0x0001, 4-byte response)
# Phase 1 (inner, dev_send_trigger_checked / FUN_0041b8d0): bit 22
# Phase 2 (outer, FUN_0040fa80 / FUN_00417b00): bits 2 and 21
_PING_MOVE_BIT   = 0x00400000   # bit 22 — axis velocity > 0, clears when motion starts settling
_PING_BUSY_MASK  = 0x00200004   # bits 2 and 21 — firmware busy, outer motion-complete gate
_PING_ERROR_MASK = 0x00100000   # bit 20

# ── Machine status flags (first 4 bytes of wValue=0x0100 block, big-endian) ──
#
# Decoded from VP_MDX40A.exe update_state_and_coords + wait_motion_complete_loop.
# Byte ordering: Python struct '>I' gives the same value as the display.
#
#  bit 27  0x08000000  SPINDLE    spindle motor on  (fires RPM/speed update)
#  bit 26  0x04000000  CMD_MOVE   motion command executing
#  bit 25  0x02000000  MTR_PWR    motor powered / parking brake released
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

FLAG_SPINDLE  = 0x08000000
FLAG_CMD_MOVE = 0x04000000
FLAG_MTR_PWR  = 0x02000000
FLAG_MOVING   = 0x00400000
FLAG_BUSY     = 0x00002000
FLAG_ERROR    = 0x00001000
FLAG_STATE    = 0x00070000
FLAG_STATE_SHIFT = 16


@dataclass
class MachineState:
    flags:  int   = 0
    x_mm:   float = 0.0    # machine coords, mm
    y_mm:   float = 0.0
    z_mm:   float = 0.0
    a_deg:  float = 0.0
    extra:  int   = 0
    raw:    bytes = field(default_factory=bytes, repr=False)

    @property
    def ready(self) -> bool:
        """True when the state block has been populated from the device."""
        return bool(self.raw)


def _decode_state(data: bytes) -> MachineState:
    data = bytes(data)
    flags = struct.unpack_from('>I', data, 0)[0]
    x, y, z, a = struct.unpack_from('>4i', data, 4)
    extra = struct.unpack_from('>I', data, 20)[0] if len(data) >= 24 else 0
    return MachineState(
        flags=flags,
        x_mm=x / 1000.0,
        y_mm=y / 1000.0,
        z_mm=z / 1000.0,
        a_deg=a / 1000.0,
        extra=extra,
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
            return struct.unpack_from('>I', data)[0]
        except usb.core.USBError:
            return -1

    def _wait_motion_complete(self, timeout: float) -> bool:
        """
        Two-phase motion-complete wait matching VP_MDX40A.exe RE.

        Phase 1 (inner, FUN_0041b8d0 inside dev_send_trigger_checked):
          Wait for ping bit 22 (0x400000) to set then clear — axis started moving
          and then velocity returned to zero.  Timeout ~3 s in firmware.

        Phase 2 (outer, FUN_0040fa80 / FUN_00417b00):
          Wait for bits 2 and 21 (0x00200004) both clear — firmware motion-complete.
          Two consecutive clear readings required.

        No motion-done ACK (SET 0x0004) is sent — that STALLs the real device.
        """
        deadline = time.monotonic() + timeout

        # ── Phase 1: wait for bit 22 to go high then come back low ──────────
        ping = self._ping_status()
        log.info("Ping after jog cmd: 0x%08X  move_bit=%s", ping & 0xFFFFFFFF, bool(ping & _PING_MOVE_BIT))

        # Wait for move bit to assert (axis has started)
        move_seen = bool(ping & _PING_MOVE_BIT)
        while not move_seen and time.monotonic() < deadline:
            time.sleep(0.050)
            ping = self._ping_status()
            if ping == -1:
                continue
            log.debug("Ph1 ping: 0x%08X", ping)
            if ping & _PING_ERROR_MASK:
                log.warning("Ping error bit in phase 1 (0x%08X)", ping)
                return False
            move_seen = bool(ping & _PING_MOVE_BIT)

        if not move_seen:
            log.warning("Phase 1: move bit never asserted — jog may not have started")
            # Fall through to phase 2 anyway; maybe move was too short to catch

        # Wait for move bit to clear (axis decelerating to stop)
        while time.monotonic() < deadline:
            time.sleep(0.050)
            ping = self._ping_status()
            if ping == -1:
                continue
            log.debug("Ph1 ping: 0x%08X  move=%s", ping, bool(ping & _PING_MOVE_BIT))
            if ping & _PING_ERROR_MASK:
                log.warning("Ping error bit in phase 1 (0x%08X)", ping)
                return False
            if not (ping & _PING_MOVE_BIT):
                break
        else:
            log.warning("Phase 1 timed out waiting for move bit to clear")
            return False

        log.debug("Phase 1 complete (move bit cleared)")

        # ── Phase 2: two consecutive reads with busy mask clear ───────────
        last_clear = False
        while time.monotonic() < deadline:
            time.sleep(0.100)
            ping = self._ping_status()
            if ping == -1:
                log.warning("Ping read failed during phase 2")
                last_clear = False
                continue
            log.debug("Ph2 ping: 0x%08X  busy=%s", ping, bool(ping & _PING_BUSY_MASK))
            if ping & _PING_ERROR_MASK:
                log.warning("Ping error bit in phase 2 (0x%08X)", ping)
                return False
            now_clear = (ping & _PING_BUSY_MASK) == 0
            if now_clear and last_clear:
                log.debug("Phase 2 complete")
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
        Use spindle_hours_minutes() for the decomposed form.
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

    @staticmethod
    def spindle_hours_minutes(seconds: int) -> tuple:
        """Decompose total spindle seconds into (hours, minutes)."""
        return seconds // 3600, (seconds // 60) % 60

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

        # Wait for ping bit 21 (0x200000) to clear — firmware ack of reset command.
        # Mirrors wait_ping_bit21_clear (FUN_0041b930) in VP_MDX40A.exe, 3 s timeout.
        _PING_BIT21 = 0x00200000
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(0.050)
            ping = self._ping_status()
            if ping == -1:
                continue
            if ping & _PING_ERROR_MASK:
                log.warning("reset_spindle_time: ping error bit set (0x%08X)", ping)
                return False
            if not (ping & _PING_BIT21):
                log.debug("Spindle reset acknowledged (bit 21 cleared)")
                return True
        log.warning("reset_spindle_time: timeout waiting for ack")
        return False

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

        # Keepalive
        try:
            with self._usb_lock:
                _usb.vend_set(dev, 0x03f5)
            log.debug("Keepalive sent")
        except usb.core.USBError as e:
            log.warning("Keepalive failed: %s", e)

    def _read_state(self) -> Optional[MachineState]:
        """Read machine state, mirroring VPanel's 200ms poll then XYZA read.

        VPanel poll (FUN_00403710): SET 0x03f5 → GET 0x3005 → GET 0x3800 → GET 0x3003 → GET 0x3b01
        These are Pattern A direct GETs (device→host with their own wValues), NOT Pattern B
        trigger reads (SET wValue + GET 0x0003). Confirmed by 0-byte GET 0x0003 responses in trace.
        XYZA coordinates are read separately (keepalive must immediately precede GET 0x0100).
        """
        try:
            with self._usb_lock:
                _usb.vend_set(self._dev, 0x03f5)
                for wv, n in ((0x3005, 8), (0x3800, 1), (0x3003, 4), (0x3b01, 4)):
                    try:
                        raw = _usb.vend_get(self._dev, wv, n)
                        if wv == 0x3800 and len(raw):
                            log.debug("GET 0x3800 status byte: 0x%02x", raw[0])
                    except usb.core.USBError as e:
                        log.debug("Poll read 0x%04x failed: %s", wv, e)
                _usb.vend_set(self._dev, 0x03f5)
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

    def _poll_loop(self) -> None:
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
            self._stop_event.wait(POLL_INTERVAL)
