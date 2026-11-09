"""Dialog titles name the active coordinate system the same way as the header (wcs_label)."""
import curses

import pytest

from rollingmill import machine as M
from rollingmill.machine import MDX40A
from rollingmill.tui import TUI, _LogBuffer
from rollingmill.usb import MdxMockUSB


class _FakeWin:
    """Just enough of a curses window to record what a dialog draws."""

    def __init__(self, h=24, w=80):
        self.h, self.w = h, w
        self.drawn = []

    def getmaxyx(self):
        return self.h, self.w

    def addstr(self, y, x, text, attr=0):
        self.drawn.append(text)

    def erase(self):
        pass

    def box(self):
        pass

    def refresh(self):
        pass

    def keypad(self, flag):
        pass

    def nodelay(self, flag):
        pass

    def move(self, y, x):
        pass

    def getch(self):
        return 27   # Esc: leave modal dialogs at once


@pytest.fixture
def fake_curses(monkeypatch):
    win = _FakeWin()
    monkeypatch.setattr(curses, "newwin", lambda *a, **k: win)
    monkeypatch.setattr(curses, "color_pair", lambda n: 0)
    monkeypatch.setattr(curses, "curs_set", lambda n: 0)
    monkeypatch.setattr(curses, "noecho", lambda: None)
    return win


@pytest.mark.parametrize("slot", range(0, 9))
def test_move_to_title_uses_the_slot_names(slot, fake_curses):
    m = MDX40A(MdxMockUSB())
    m.set_active_wcs(slot)
    TUI(m, _LogBuffer())._draw_move_dialog(_FakeWin(), 30, 100)
    title = next(t for t in fake_curses.drawn if "Move To" in t)
    assert title.strip() == f"Move To  (using {M.WCS_SLOT_NAMES[slot]})"


def test_move_to_title_for_user_and_g54_reads_naturally(fake_curses):
    m = MDX40A(MdxMockUSB())
    for slot, name in ((1, "User (RML-1)"), (3, "WCS1 (G54)")):
        fake_curses.drawn.clear()
        m.set_active_wcs(slot)
        TUI(m, _LogBuffer())._draw_move_dialog(_FakeWin(), 30, 100)
        assert any(name in t for t in fake_curses.drawn)


@pytest.mark.parametrize("slot", range(0, 9))
def test_move_to_position_title_uses_the_slot_names(slot, fake_curses):
    m = MDX40A(MdxMockUSB())
    m.set_active_wcs(slot)
    TUI(m, _LogBuffer())._coord_entry_dialog(_FakeWin())
    title = next(t for t in fake_curses.drawn if "Move to position" in t)
    assert title.strip() == f"Move to position ({M.WCS_SLOT_NAMES[slot]})"
