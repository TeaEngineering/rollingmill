"""Replay a real captured descent/retract plus failure and UI interlock paths."""
import json
import struct
from pathlib import Path

import pytest
import usb.core

from rollingmill.machine import FLAG_DOOR, FLAG_ERROR, MDX40A, _decode_state
from rollingmill.probing import ZProbe
from rollingmill.tui import TUI, _LogBuffer
from rollingmill.usb import MdxMockUSB, MdxUSB

IDLE = bytes.fromhex("00 92 08 1c 00 01 02 0a 00 00 c5 51 ff fe fc ed 00 00 00 00 00 00 11 94 00 00 64 64 64 00 00 00")


class Link(MdxMockUSB):
    def __init__(self, sensor=b"\x00\x14"):
        super().__init__({0x0100: IDLE, 0x2001: sensor, 0x030b: struct.pack(">4i", 66420, 161650, -94960, 8600)})
        self.sent = []
        self.fail = None

    def vend_set(self, cmd, data=b""):
        self.sent.append((cmd, data))
        if cmd == self.fail:
            raise usb.core.USBError("test disconnect")
        super().vend_set(cmd, data)


def fixture(sensor=b"\x00\x14"):
    link = Link(sensor)
    m = MDX40A(link)
    m.set_active_wcs(1)
    link.sent.clear()
    clock = [0.0]
    probe = ZProbe(m, clock=lambda: clock[0])
    return link, m, probe, clock


def test_live_capture_replay_and_exact_wire_packet():
    link, m, probe, clock = fixture()
    probe.start()
    assert (0x3902, bytes.fromhex("00 01 ff ff ff ff")) in link.sent
    assert [c for c, _ in link.sent].count(0x2001) == 2
    # The real USB layer adds exactly the captured driver header.
    class Device:
        def ctrl_transfer(self, *args, **kwargs):
            self.args = args
    dev = Device()
    MdxUSB(dev, 0, None, 2000).vend_set(0x3902, bytes.fromhex("00 01 ff ff ff ff"))
    assert dev.args == (0x40, 1, 0x3902, 0, bytes.fromhex("01 39 02 00 00 01 ff ff ff ff"))
    probe.update(_decode_state(IDLE))
    assert probe.active  # idle-before-ack is not completion
    samples = json.loads((Path(__file__).parent / "fixtures/mdx40a_zprobe_status.json").read_text())
    moving = False
    for data in samples:
        state = _decode_state(bytes(data))
        if state.flags & 0x04000000:
            moving = True
        if moving:
            clock[0] += 0.2
            probe.update(state)
            if not probe.active:
                break
    assert moving and not probe.active
    assert "cycle ended" in probe.message
    assert m.wcs_offset[2] == pytest.approx(-94.960)
    assert link.sent[-3:] == [(0x03f5, b"\xff"), (0x1109, b"\xff"), (0x030b, b"")]


@pytest.mark.parametrize("sensor,match", [(b"\0\0", "not connected"), (b"\0\x1c", "already in contact"), (b"\0", "Short")])
def test_reject_sensor_without_motion(sensor, match):
    link, _, probe, _ = fixture(sensor)
    # Preserve short reply (default mock pads responses).
    original = probe.machine.pattern_b_read
    probe.machine.pattern_b_read = lambda cmd, n: sensor if cmd == 0x2001 else original(cmd, n)
    with pytest.raises(ValueError, match=match):
        probe.start()
    assert not probe.active
    assert not any(cmd == 0x3902 for cmd, _ in link.sent)
    assert link.sent[-1] == (0x1109, b"\xff")


@pytest.mark.parametrize("flag", [FLAG_DOOR, FLAG_ERROR])
def test_reject_unsafe_state(flag):
    link, _, probe, _ = fixture()
    raw = bytearray(IDLE)
    raw[:4] = (int.from_bytes(raw[:4], 'big') | flag).to_bytes(4, 'big')
    link._responses[0x0100] = bytes(raw)
    with pytest.raises(ValueError):
        probe.start()
    assert not any(c == 0x3902 for c, _ in link.sent)


def test_unsupported_wcs_no_writes():
    link, m, probe, _ = fixture()
    m.set_active_wcs(3)
    link.sent.clear()
    with pytest.raises(ValueError, match="RML-1"):
        probe.start()
    assert link.sent == []


def test_no_motion_timeout_is_not_success():
    _, _, probe, clock = fixture()
    probe.start()
    clock[0] = 4
    probe.update(_decode_state(IDLE))
    assert not probe.active
    assert "unverified" in probe.message


def test_lost_status_cancel_and_wait_for_fresh_idle():
    link, _, probe, clock = fixture()
    probe.start()
    clock[0] = 4
    probe.update(None)
    assert probe.active and (0x03f3, b"") in link.sent
    probe.update(_decode_state(IDLE))
    assert not probe.active and "unverified" in probe.message


def test_motion_write_failure_remains_locked_until_idle():
    link, _, probe, _ = fixture()
    link.fail = 0x3902
    with pytest.raises(usb.core.USBError):
        probe.start()
    assert probe.active and (0x03f3, b"") in link.sent
    probe.update(_decode_state(IDLE))
    assert not probe.active and "unverified" in probe.message


def test_tui_confirmation_and_motion_interlocks():
    link, m, _, _ = fixture()
    tui = TUI(m, _LogBuffer())
    tui._handle_key(ord('b'))
    assert tui._probe_confirm and not any(c == 0x3902 for c, _ in link.sent)
    tui._handle_key(27)
    assert not tui._probe_confirm
    tui._handle_key(ord('b'))
    tui._handle_key(ord('y'))
    assert tui._probe.active
    n = len(link.sent)
    for key in ('z', 's', 'c', 'r', 'b'):
        tui._handle_key(ord(key))
    assert len(link.sent) == n
    tui._handle_key(27)
    assert (0x03f3, b"") in link.sent
    assert tui._probe.active  # remain locked until a fresh idle response


def test_active_poll_has_no_idle_heartbeat_or_extra_queries():
    link, _, probe, _ = fixture()
    probe.start()
    link.sent.clear()
    assert probe.poll() is not None
    assert link.sent == []


def test_cancel_keeps_controls_locked_while_moving():
    _, _, probe, _ = fixture()
    probe.start()
    moving = bytearray(IDLE)
    moving[0] = 6
    moving[1] = 0xd2
    moving[2] = 0x28
    probe.cancel()
    probe.update(_decode_state(bytes(moving)))
    assert probe.active
    probe.update(_decode_state(IDLE))
    assert not probe.active and "unverified" in probe.message
