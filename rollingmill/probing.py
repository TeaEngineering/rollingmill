"""Cooperative Z0 sensor cycle captured on a real MDX-40A.

The cycle sets the Z origin of the active coordinate system: User/RML-1 (slot 1,
the captured cycle), EXOFS (slot 2) or G54-G59 (slots 3-8). The first word of
SET 0x3902 is the slot index, as VPanel's MDx3902_GiantDispatch encodes it (see
docs/z-probing.md). No sensor thickness or uncaptured settings are written.
"""
import logging
import struct
import time

import usb.core

from .machine import (FLAG_CMD_MOVE, FLAG_DOOR, FLAG_ERROR, FLAG_MOVING, FLAG_SPINDLE,
                      _decode_state, wcs_label)

log = logging.getLogger(__name__)
MOTION = FLAG_CMD_MOVE | FLAG_MOVING | 0x00002000

# WCS slots the probe can target: 1 = User (RML-1), 2 = EXOFS, 3-8 = G54-G59. Only slot 1
# has a live capture. VPanel only offers slots 2-8 when the command set is NC Code or automatic.
PROBE_SLOTS = range(1, 9)


class ZProbe:
    """Call update with each *fresh* poll result; UI remains responsive."""

    def __init__(self, machine, clock=time.monotonic):
        self.machine = machine
        self.clock = clock
        self.active = False
        self.message = ""
        self.slot = 0  # WCS slot being probed, set by start()
        self._bracket = False
        self._seen_motion = False
        self._cancelled = False
        self._started = 0.0
        self._last_fresh = 0.0

    def start(self):
        if self.active:
            raise ValueError("A Z-probe cycle is already active")
        slot = self.machine.active_wcs
        if slot not in PROBE_SLOTS:
            raise ValueError(
                "Select User (RML-1), EXOFS or G54-G59 in the coordinate dialog first "
                f"(active: {wcs_label(slot)})"
            )
        state = self.machine.poll()
        if state is None or not state.idle or self.machine.is_busy:
            raise ValueError("Fresh idle machine status is required")
        if state.flags & (FLAG_DOOR | FLAG_ERROR | FLAG_CMD_MOVE | FLAG_MOVING):
            raise ValueError("Close the cover and clear machine errors/motion first")
        if state.flags & FLAG_SPINDLE:
            raise ValueError("Turn the spindle off before probing")
        link = self.machine._link
        try:
            link.vend_set(0x03F5, b"\x02")
            # Track before writing: an ambiguous USB failure may still reach firmware.
            self._bracket = True
            link.vend_set(0x1109, b"\x00")
            link.vend_set(0x3006, struct.pack(">I", 0))
            for _ in range(2):
                sensor = self.machine.pattern_b_read(0x2001, 2)
                if len(sensor) != 2:
                    raise ValueError("Short Z0 sensor reply")
                if not sensor[1] & 0x04:
                    raise ValueError("Z0 sensor is not connected")
                if sensor[1] & 0x08:
                    raise ValueError("Z0 sensor is already in contact with the tool")
            self.slot = slot
            self._started = self._last_fresh = self.clock()
            self._seen_motion = self._cancelled = False
            self.active = True
            self.message = f"Z-probe of {wcs_label(slot)} starting; Esc stops motion"
            link.vend_set(0x3902, struct.pack(">HHH", slot, 0xFFFF, 0xFFFF))
        except (ValueError, usb.core.USBError):
            if self.active:
                # Motion write outcome is unknown. Keep UI locked until fresh idle.
                self.cancel("Probe command failed; stopping, outcome unverified")
            else:
                self._finish_bracket()
            raise
        log.info(self.message)

    def _finish_bracket(self):
        if self._bracket:
            # Try both even if one fails. Do not report a transport failure as success.
            errors = []
            for cmd, data in ((0x03F5, b"\xff"), (0x1109, b"\xff")):
                try:
                    self.machine._link.vend_set(cmd, data)
                except usb.core.USBError as exc:
                    errors.append(str(exc))
            self._bracket = False
            if errors:
                raise ValueError("Probe cleanup failed: " + "; ".join(errors))

    def cancel(self, reason="Z-probe cancelled; waiting for motion to stop"):
        if self.active and not self._cancelled:
            self._cancelled = True
            self.machine.stop_motion()
            self.message = reason
            log.warning(reason)

    def poll(self):
        """During the cycle use only the captured status reads, not idle heartbeat writes."""
        try:
            raw = self.machine._link.vend_get(0x0100, 32)
            if len(raw) != 32:
                return None
            state = _decode_state(raw)
            self.machine._state = state
            self.machine._ping_status()
            return state
        except usb.core.USBError:
            return None

    def update(self, state):
        if not self.active:
            return
        now = self.clock()
        if state is None:
            if now - self._last_fresh > 3:
                self.cancel("Status lost; stop requested, waiting for reconnection")
            return
        self._last_fresh = now
        if state.flags & FLAG_ERROR:
            self.cancel("Machine error during probing; stop requested")
        busy = bool(state.flags & MOTION)
        self._seen_motion |= busy
        if now - self._started > 120:
            self.cancel("Z-probe timed out; stop requested")
        if not self._seen_motion and now - self._started > 3:
            self.cancel("Probe motion was not observed; result unverified")
        if busy or not state.idle:
            if not self._cancelled:
                feed = int.from_bytes(state.raw[24:26], "big") if len(state.raw) >= 26 else 0
                self.message = f"Z-probe running: Z {state.z_mm:.3f} mm, feed {feed} mm/min; Esc stops"
            return
        if not self._cancelled and not self._seen_motion:
            return  # Never treat the first still-idle poll as completion.
        try:
            self._finish_bracket()
            if self._cancelled:
                self.message = "Z-probe stopped; origin result unverified"
            else:
                origin = self.machine.get_wcs_origin(self.slot)
                if origin is None:
                    raise ValueError(f"Cycle ended but {wcs_label(self.slot)} origin could not be refreshed")
                if self.machine.active_wcs == self.slot:
                    self.machine._wcs_offset = origin
                self.message = f"Z-probe cycle ended; {wcs_label(self.slot)} Z origin {origin[2]:.3f} mm"
            log.info(self.message)
        except (ValueError, usb.core.USBError) as exc:
            self.message = f"Z-probe result unverified: {exc}"
            log.error(self.message)
        self.active = False

    def close(self):
        """Best-effort stop/cleanup when the application exits unexpectedly."""
        if self.active:
            self.cancel("Application exiting; Z-probe stop requested")
            self._finish_bracket()
            self.active = False
