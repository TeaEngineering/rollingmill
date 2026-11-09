"""'M' in the coordinates dialog moves to the selected coordinate system's origin, MCS included."""
import struct

import pytest

from rollingmill.machine import MDX40A
from rollingmill.tui import TUI, _LogBuffer
from rollingmill.usb import MdxMockUSB

DOWN = 258   # curses.KEY_DOWN


class _Recording(MdxMockUSB):
    def __init__(self, responses=None):
        super().__init__(responses=responses)
        self.sent = []

    def vend_set(self, wValue, data=b""):
        self.sent.append((wValue, data))
        super().vend_set(wValue, data)

    def moves(self):
        return [d for w, d in self.sent if w == 0x04f7]


def _dialog(responses=None):
    link = _Recording(responses)
    tui = TUI(MDX40A(link), _LogBuffer())
    tui._wcs_open_dialog()
    link.sent.clear()
    return tui, link


def _move_payload(x, y, z, a, speed=1800):
    return struct.pack(">HH4i", speed, 0xFFFF, round(x * 1000), round(y * 1000), round(z * 1000), round(a * 1000))


def test_m_on_mcs_sends_a_move_to_machine_zero():
    tui, link = _dialog()
    assert tui._wcs_sel == 0                                   # MCS is the first row
    tui._handle_key(ord("m"))
    assert link.moves() == [_move_payload(0, 0, 0, 0)]


def test_m_on_mcs_is_bracketed_like_every_other_move():
    tui, link = _dialog()
    tui._handle_key(ord("M"))
    assert [w for w, _ in link.sent] == [0x1109, 0x04f7, 0x03f2, 0x1109]
    assert link.sent[0][1] == b"\x00" and link.sent[-1][1] == b"\xff"


def test_m_on_a_stored_origin_moves_to_it():
    tui, link = _dialog({0x3203: struct.pack(">4i", 1000, 2000, -3000, 4000)})   # G54 (slot 3)
    for _ in range(3):
        tui._handle_key(DOWN)
    tui._handle_key(ord("m"))
    assert link.moves() == [_move_payload(1, 2, -3, 4)]


@pytest.mark.parametrize("key", [ord("x"), ord("y"), ord("z"), ord("a"), ord("0")])
def test_the_overwrite_keys_still_refuse_mcs(key):
    tui, link = _dialog()
    tui._handle_key(key)
    assert link.sent == []                                     # MCS origin cannot be overwritten
