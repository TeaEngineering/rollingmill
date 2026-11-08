"""The command mode dialog ('n'), the header label and the cached-mode probe warning."""
import logging

import pytest
import usb.core

from rollingmill import machine as M
from rollingmill.machine import MDX40A
from rollingmill.tui import _CMD_ROWS, TUI, _cmd_mode_label, _LogBuffer, _probe_mode_warning
from rollingmill.usb import MdxMockUSB

KEY_UP, KEY_DOWN, KEY_RIGHT = 259, 258, 261   # curses.KEY_*
SPACE, ENTER, ESC = 32, 10, 27


class _Config(MdxMockUSB):
    """Mock whose GET 0x3106 follows SET 0x3107, like the firmware (unless it refuses writes)."""

    def __init__(self, raw=bytes([0, 1, 0, 0, 0, 5]), accept=True):
        super().__init__(responses={0x3106: raw})
        self.sent = []
        self.accept = accept

    def vend_set(self, wValue, data=b""):
        self.sent.append((wValue, data))
        if wValue == 0x3107 and self.accept:
            self._responses[0x3106] = data
        super().vend_set(wValue, data)

    def writes(self):
        return [d for w, d in self.sent if w == 0x3107]


def _tui(raw=bytes([0, 1, 0, 0, 0, 5]), **kw):
    link = _Config(raw, **kw)
    return TUI(MDX40A(link), _LogBuffer()), link


def _row(field, value):
    return next(i for i, (f, v, _l, _n) in enumerate(_CMD_ROWS) if (f, v) == (field, value))


def _go_to(tui, field, value):
    """Move the cursor to an option row using the arrow keys, as a user would."""
    target = _row(field, value)
    while tui._cmd_sel != target:
        before = tui._cmd_sel
        tui._handle_key(KEY_DOWN if target > tui._cmd_sel else KEY_UP)
        assert tui._cmd_sel != before, f"cursor stuck at {before}, cannot reach row {target}"


def _select(tui, field, value):
    """Move to an option row and select it with Space."""
    _go_to(tui, field, value)
    tui._handle_key(SPACE)


# ── header label and probe warning ───────────────────────────────────────────

@pytest.mark.parametrize("mode,label", [(M.CMDSET_RML1, "RML-1 mode"), (M.CMDSET_NC, "NC mode"),
                                        (M.CMDSET_AUTO, "Auto-mode"), (None, "mode ?"), (7, "mode ?")])
def test_header_label(mode, label):
    assert _cmd_mode_label(mode) == label


@pytest.mark.parametrize("slot,mode,warns", [
    (1, M.CMDSET_RML1, False), (1, M.CMDSET_AUTO, False), (1, M.CMDSET_NC, True),     # VPanel hides User in NC
    (2, M.CMDSET_RML1, True), (3, M.CMDSET_RML1, True), (8, M.CMDSET_RML1, True),     # ...and G54-G59/EXOFS in RML-1
    (3, M.CMDSET_NC, False), (8, M.CMDSET_AUTO, False), (2, M.CMDSET_NC, False),
    (1, None, False), (3, None, False),                                               # unknown: no claim
])
def test_probe_mode_warning(slot, mode, warns):
    assert (_probe_mode_warning(slot, mode) is not None) is warns


def test_probe_dialog_uses_the_cached_command_set_without_reading_it_again():
    tui, link = _tui()
    tui._m.set_active_wcs(1)
    link.sent.clear()
    tui._handle_key(ord('b'))
    assert tui._probe_confirm
    assert (0x3106, b"") not in link.sent


# ── opening ──────────────────────────────────────────────────────────────────

def test_n_opens_with_the_machine_values_and_the_cursor_on_the_current_command_set():
    tui, _ = _tui(bytes([0, 2, 3, 1, 1, 5]))
    tui._handle_key(ord('n'))
    assert tui._cmd_open
    assert tui._cmd_vals == {'command_set': 2, 'offset_type': 1, 'decimal_method': 3, 'optional_block': 1}
    assert _CMD_ROWS[tui._cmd_sel][:2] == ('command_set', M.CMDSET_AUTO)


def test_n_is_refused_while_a_job_is_loaded(caplog):
    tui, link = _tui()
    tui._cut_job = object()
    with caplog.at_level(logging.WARNING):
        tui._handle_key(ord('n'))
    assert not tui._cmd_open
    assert any(r.name == "rollingmill.tui" and "command mode" in r.getMessage() for r in caplog.records)


def test_n_does_not_open_when_the_config_cannot_be_read(caplog):
    class _Failing(_Config):
        def vend_get(self, wValue, length):
            if wValue == 0x0001 and self.fail:
                raise usb.core.USBError("boom")
            return super().vend_get(wValue, length)

    link = _Failing()
    link.fail = False
    tui = TUI(MDX40A(link), _LogBuffer())
    link.fail = True
    with caplog.at_level(logging.WARNING):
        tui._handle_key(ord('n'))
    assert not tui._cmd_open
    assert any("could not read" in r.getMessage() for r in caplog.records)


# ── navigation: NC Code preferences follow the command set ───────────────────

def test_nc_preferences_are_skipped_by_the_cursor_in_rml1_mode():
    tui, _ = _tui(bytes([0, 0, 0, 0, 0, 5]))               # RML-1
    tui._handle_key(ord('n'))
    tui._handle_key(KEY_DOWN)
    tui._handle_key(KEY_DOWN)
    tui._handle_key(KEY_DOWN)                              # past the last command-set row: nowhere to go
    assert _CMD_ROWS[tui._cmd_sel][0] == 'command_set'


def test_choosing_nc_code_unlocks_the_preferences():
    tui, _ = _tui(bytes([0, 0, 0, 0, 0, 5]))
    tui._handle_key(ord('n'))
    _go_to(tui, 'command_set', M.CMDSET_NC)
    tui._handle_key(SPACE)
    _go_to(tui, 'decimal_method', M.DECIMAL_CALC_F)       # reachable now
    assert _CMD_ROWS[tui._cmd_sel][:2] == ('decimal_method', M.DECIMAL_CALC_F)


def test_space_and_right_arrow_select_but_do_not_write():
    tui, link = _tui(bytes([0, 1, 0, 0, 0, 5]))
    tui._handle_key(ord('n'))
    link.sent.clear()
    _go_to(tui, 'offset_type', M.OFFSET_TYPE_B)
    tui._handle_key(KEY_RIGHT)
    assert tui._cmd_vals['offset_type'] == M.OFFSET_TYPE_B
    _go_to(tui, 'optional_block', M.OPTIONAL_BLOCK_DISABLED)
    tui._handle_key(SPACE)
    assert tui._cmd_vals['optional_block'] == M.OPTIONAL_BLOCK_DISABLED
    assert link.writes() == []


# ── applying ─────────────────────────────────────────────────────────────────

def test_enter_writes_the_changed_fields_in_one_set_and_reloads():
    tui, link = _tui(bytes([0, 1, 0, 0, 0, 5]))           # NC Code, all defaults
    tui._handle_key(ord('n'))
    _select(tui, 'offset_type', M.OFFSET_TYPE_B)
    _select(tui, 'decimal_method', M.DECIMAL_CALC_IJKRXYZA)
    _select(tui, 'optional_block', M.OPTIONAL_BLOCK_DISABLED)
    tui._handle_key(ENTER)
    assert link.writes() == [bytes([0, 1, 3, 1, 1, 5])]    # bytes 2, 3, 4 changed; 0, 1, 5 preserved
    assert tui._cmd_notice == 'Applied'
    assert tui._cmd_open                                   # stays open, now showing the new values
    assert tui._cmd_vals == {'command_set': 1, 'offset_type': 1, 'decimal_method': 3, 'optional_block': 1}


def test_switching_command_set_writes_only_byte_1():
    tui, link = _tui(bytes([0, 0, 2, 1, 1, 5]))
    tui._handle_key(ord('n'))
    _select(tui, 'command_set', M.CMDSET_AUTO)
    tui._handle_key(ENTER)
    assert link.writes() == [bytes([0, 2, 2, 1, 1, 5])]
    assert tui._m.command_set == M.CMDSET_AUTO            # header label follows


def test_enter_without_changes_writes_nothing():
    tui, link = _tui()
    tui._handle_key(ord('n'))
    link.sent.clear()
    tui._handle_key(ENTER)
    assert link.writes() == [] and tui._cmd_notice == 'No changes to apply'


def test_esc_closes_without_writing():
    tui, link = _tui()
    tui._handle_key(ord('n'))
    _select(tui, 'offset_type', M.OFFSET_TYPE_B)
    link.sent.clear()
    tui._handle_key(ESC)
    assert not tui._cmd_open and link.writes() == []


def test_a_write_the_machine_does_not_keep_is_reported_and_the_real_values_shown():
    tui, link = _tui(bytes([0, 1, 0, 0, 0, 5]), accept=False)
    tui._handle_key(ord('n'))
    _select(tui, 'offset_type', M.OFFSET_TYPE_B)
    tui._handle_key(ENTER)
    assert tui._cmd_notice.startswith('Apply failed')
    assert tui._cmd_vals['offset_type'] == M.OFFSET_TYPE_A


def test_other_keys_do_not_leak_through_the_open_dialog():
    tui, link = _tui()
    tui._handle_key(ord('n'))
    link.sent.clear()
    for key in (ord('s'), ord('m'), ord('t'), ord('c'), ord('b'), ord('z')):
        tui._handle_key(key)
    assert tui._cmd_open and link.sent == []
