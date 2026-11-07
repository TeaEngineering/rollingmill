import curses
import struct
from unittest.mock import Mock

import pytest
from test_probing import IDLE, Link

from rollingmill.machine import FLAG_DOOR, MDX40A, _decode_state
from rollingmill.tui import TUI, _LogBuffer


@pytest.mark.parametrize("slot", range(9))
@pytest.mark.parametrize("mask", [1, 2, 3, 4, 8])
def test_move_origin_uses_selected_coordinates_and_preserves_other_axes(slot, mask):
    link = Link()
    machine = MDX40A(link)
    machine._active_wcs = slot
    machine.get_wcs_origin = Mock(return_value=(11.0, 22.0, 33.0, 44.0))
    machine.poll = Mock(return_value=_decode_state(IDLE))
    machine.move_to_origin(mask, speed=1800)
    current = _decode_state(IDLE)
    origin = (0, 0, 0, 0) if slot == 0 else (11000, 22000, 33000, 44000)
    position = [round(v * 1000) for v in (current.x_mm, current.y_mm, current.z_mm, current.a_deg)]
    expected = [origin[i] if mask & (1 << i) else position[i] for i in range(4)]
    assert (0x04f7, struct.pack('>HH4i', 1800, 0xffff, *expected)) in link.sent
    assert not any(cmd == 0x3501 for cmd, _ in link.sent)
    if slot:
        machine.get_wcs_origin.assert_called_once_with(slot)


def test_origin_read_failure_sends_no_motion():
    link = Link()
    machine = MDX40A(link)
    machine._active_wcs = 4
    link.sent.clear()
    machine.get_wcs_origin = Mock(return_value=None)
    with pytest.raises(ValueError):
        machine.move_to_origin(3)
    assert not link.sent


def test_continuous_refresh_during_motion_repeat_and_stop(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('rollingmill.tui.time.monotonic', lambda: clock[0])
    link = Link()
    machine = MDX40A(link)
    machine.poll = Mock(return_value=_decode_state(IDLE))
    machine._state = _decode_state(IDLE)
    tui = TUI(machine, _LogBuffer())
    tui._held_keys = Mock(window=123, event_type=2)
    tui._held_keys.held.return_value = True
    tui._handle_key(curses.KEY_RIGHT)
    assert tui._continuous_axis == 'X'
    initial = sum(c == 0x04f5 for c, _ in link.sent)
    tui._handle_key(curses.KEY_RIGHT)
    assert sum(c == 0x04f5 for c, _ in link.sent) == initial
    machine._state.flags |= 0x04400000
    clock[0] += 0.11
    tui._service_continuous_jog()
    assert sum(c == 0x04f5 for c, _ in link.sent) == initial + 1
    tui._handle_key(ord('m'))
    assert not tui._move_open
    tui._handle_key(27)
    assert tui._continuous_axis is None
    assert (0x03f3, b'') in link.sent
    assert link.sent[-1] == (0x1109, b'\xff')


@pytest.mark.parametrize('reason', ['stale', 'door'])
def test_continuous_stops_on_status_loss_or_open_cover(monkeypatch, reason):
    clock = [10.0]
    monkeypatch.setattr('rollingmill.tui.time.monotonic', lambda: clock[0])
    link = Link()
    machine = MDX40A(link)
    machine.poll = Mock(return_value=_decode_state(IDLE))
    machine._state = _decode_state(IDLE)
    tui = TUI(machine, _LogBuffer())
    tui._held_keys = Mock(window=123, event_type=2)
    tui._held_keys.held.return_value = True
    tui._continuous = True
    tui._start_jog('Z', -1)
    if reason == 'stale':
        clock[0] += 1
    else:
        machine._state.flags |= FLAG_DOOR
    tui._service_continuous_jog()
    assert tui._continuous_axis is None
    assert (0x03f3, b'') in link.sent


def test_release_stops_motion(monkeypatch):
    link = Link()
    machine = MDX40A(link)
    machine.poll = Mock(return_value=_decode_state(IDLE))
    machine._state = _decode_state(IDLE)
    tui = TUI(machine, _LogBuffer())
    tui._held_keys = Mock(window=123, event_type=2)
    tui._held_keys.held.return_value = True
    tui._continuous = True
    tui._start_jog('X', 1)
    tui._held_keys.held.return_value = False
    tui._service_continuous_jog()
    assert tui._continuous_axis is None
    assert (0x03f3, b'') in link.sent


def test_terminal_release_focus_and_opposing_keys():
    from rollingmill.keyrelease import HeldKeys
    keys = HeldKeys()
    keys.feed("\x1b[?11u\x1b[57351;1:1u")
    assert keys.supported and keys.held('X', 1, 1)
    keys.feed("\x1b[57351;1:2u")
    assert keys.held('X', 1, 1)
    keys.feed("\x1b[57351;1:3u")
    assert not keys.held('X', 1, 1)
    keys.feed("\x1b[97;1:1u\x1b[122;1:1u")
    assert not keys.held('Z', 1, 1)
    keys.feed("\x1b[122;1:3u")
    assert keys.held('Z', 1, 1)
    keys.feed("\x1b[O")
    assert not keys.held('Z', 1, 1)


def test_legacy_terminal_does_not_enable_continuous_motion():
    from rollingmill.keyrelease import HeldKeys
    keys = HeldKeys()
    keys.feed("\x1b[Caz")
    assert not keys.supported
    assert not keys.held('X', 1, None)


def test_arrow_release_uses_csi_direction_encoding():
    from rollingmill.keyrelease import HeldKeys
    keys = HeldKeys()
    keys.feed("\x1b[?11u\x1b[1;1:1C")
    assert keys.held('X', 1, 1)
    keys.feed("\x1b[1;1:2C")
    assert keys.held('X', 1, 1)
    keys.feed("\x1b[1;1:3C")
    assert not keys.held('X', 1, 1)


def legacy_tui(monkeypatch):
    from rollingmill.keyrelease import HeldKeys
    clock = [10.0]
    monkeypatch.setattr('rollingmill.tui.time.monotonic', lambda: clock[0])
    link = Link()
    machine = MDX40A(link)
    machine.poll = Mock(return_value=_decode_state(IDLE))
    machine._state = _decode_state(IDLE)
    tui = TUI(machine, _LogBuffer())
    tui._held_keys = HeldKeys()
    tui._pulse_i = 1
    link.sent.clear()
    return tui, link, clock


def key_at(tui, clock, when, key=curses.KEY_RIGHT):
    clock[0] = when
    tui._held_keys.event_time = when
    tui._handle_key(key)


def test_ordinary_terminal_tap_is_one_bounded_step(monkeypatch):
    tui, link, clock = legacy_tui(monkeypatch)
    key_at(tui, clock, 10.0)
    assert not tui._continuous_axis
    assert link.sent == [(0x04f5, struct.pack('>HH4i', 60, 0, 100, 0, 0, 0))]
    clock[0] = 10.3
    tui._update_jog_indicator()
    tui._service_continuous_jog()
    assert len(link.sent) == 1


def test_repeat_delay_refresh_and_200ms_release_timeout(monkeypatch):
    tui, link, clock = legacy_tui(monkeypatch)
    key_at(tui, clock, 10.0)
    clock[0] = 10.3
    tui._update_jog_indicator()
    # Standard keyboard initial-repeat delay, not a 200 ms initial deadline.
    key_at(tui, clock, 10.6)
    assert tui._continuous_axis == 'X'
    count = sum(c == 0x04f5 for c, _ in link.sent)
    key_at(tui, clock, 10.65)
    clock[0] = 10.71
    tui._service_continuous_jog()
    assert sum(c == 0x04f5 for c, _ in link.sent) == count + 1
    clock[0] = 10.84
    tui._service_continuous_jog()
    assert tui._continuous_axis == 'X'
    clock[0] = 10.86
    tui._service_continuous_jog()
    assert tui._continuous_axis is None
    assert link.sent[-2:] == [(0x03f3, b''), (0x1109, b'\xff')]


def test_escape_cannot_be_undone_by_remaining_repeats(monkeypatch):
    tui, link, clock = legacy_tui(monkeypatch)
    key_at(tui, clock, 10.0)
    clock[0] = 10.3
    tui._update_jog_indicator()
    key_at(tui, clock, 10.6)
    key_at(tui, clock, 10.65, 27)
    count = sum(c == 0x04f5 for c, _ in link.sent)
    for when in (10.7, 10.75, 10.8, 10.85):
        key_at(tui, clock, when)
    assert tui._continuous_axis is None
    assert sum(c == 0x04f5 for c, _ in link.sent) == count
    key_at(tui, clock, 11.2)
    assert sum(c == 0x04f5 for c, _ in link.sent) == count + 1
    assert tui._continuous_axis is None  # A new press is a bounded tap first.


def test_old_buffered_repeat_cannot_renew_motion(monkeypatch):
    tui, link, clock = legacy_tui(monkeypatch)
    key_at(tui, clock, 10.0)
    clock[0] = 10.3
    tui._update_jog_indicator()
    key_at(tui, clock, 10.6)
    clock[0] = 10.85
    tui._held_keys.event_time = 10.64
    tui._handle_key(curses.KEY_RIGHT)
    tui._service_continuous_jog()
    assert tui._continuous_axis is None


def test_slow_repeat_stops_instead_of_latching_motion(monkeypatch):
    tui, link, clock = legacy_tui(monkeypatch)
    key_at(tui, clock, 10.0)
    clock[0] = 10.3
    tui._update_jog_indicator()
    key_at(tui, clock, 10.6)
    clock[0] = 10.81
    tui._service_continuous_jog()
    assert tui._continuous_axis is None


def test_step_mode_still_uses_selected_distance(monkeypatch):
    tui, link, clock = legacy_tui(monkeypatch)
    tui._continuous = False
    tui._pulse_i = 1
    key_at(tui, clock, 10.0)
    assert link.sent == [(0x04f5, struct.pack('>HH4i', 60, 0, 100, 0, 0, 0))]


@pytest.mark.parametrize('index,pulse,feed', [(0, 0.01, 6), (1, 0.1, 60), (2, 1.0, 600), (3, 5.0, 3000)])
@pytest.mark.parametrize('axis,sign', [('X', 1), ('Z', -1), ('A', 1)])
def test_selected_pulse_exact_payload_for_tap_and_constant_hold(monkeypatch, index, pulse, feed, axis, sign):
    tui, link, clock = legacy_tui(monkeypatch)
    tui._pulse_i = index
    tui._start_jog(axis, sign)
    vector = [0, 0, 0, 0]
    vector['XYZA'.index(axis)] = sign * round(pulse * 1000)
    packet = (0x04f5, struct.pack('>HH4i', feed, 0, *vector))
    assert link.sent == [packet]
    clock[0] = 10.3
    tui._update_jog_indicator()
    clock[0] = 10.6
    tui._start_jog(axis, sign)
    assert tui._continuous_axis == axis
    # All pulses, including the first held pulse, have exactly the same feed.
    for tick in range(1, 50):
        clock[0] = 10.6 + tick * 0.11
        tui._jog_last_fresh = clock[0]
        tui._start_jog(axis, sign)  # Renew standard-terminal input lease.
        tui._service_continuous_jog()
    pulses = [sent for sent in link.sent if sent[0] == 0x04f5]
    assert len(pulses) == 51
    assert all(sent == packet for sent in pulses)


def test_f_cycles_only_the_four_pulse_feed_pairs(monkeypatch):
    from rollingmill.tui import JOG_PULSES
    tui, _, _ = legacy_tui(monkeypatch)
    tui._pulse_i = 0
    values = []
    for _ in range(4):
        values.append(JOG_PULSES[tui._pulse_i])
        tui._handle_key(ord('f'))
    assert values == [(0.01, 6), (0.1, 60), (1.0, 600), (5.0, 3000)]
    assert tui._pulse_i == 0
    tui._handle_key(ord('5'))
    assert tui._pulse_i == 0  # Previous independent step selector removed.


def test_protocol_tap_also_uses_selected_pulse(monkeypatch):
    tui, link, clock = legacy_tui(monkeypatch)
    tui._pulse_i = 3
    tui._held_keys.feed("\x1b[?11u\x1b[97;1:1u")
    tui._held_keys._pop()
    tui._handle_key(ord('a'))
    assert tui._continuous_axis is None
    assert link.sent == [(0x04f5, struct.pack('>HH4i', 3000, 0, 0, 0, 5000, 0))]
    clock[0] = 10.3
    tui._update_jog_indicator()
    tui._held_keys.feed("\x1b[97;1:2u")
    tui._held_keys._pop()
    tui._handle_key(ord('a'))
    assert tui._continuous_axis == 'Z'
    tui._held_keys.feed("\x1b[97;1:3u")
    tui._service_continuous_jog()
    assert tui._continuous_axis is None


@pytest.mark.parametrize('slot', range(9))
def test_numeric_position_uses_selected_frame_and_blank_axes_are_unchanged(slot):
    link = Link()
    machine = MDX40A(link)
    machine.get_wcs_origin = Mock(return_value=(11.0, 22.0, 33.0, 44.0))
    machine.poll = Mock(return_value=_decode_state(IDLE))
    machine.set_active_wcs(slot)
    link.sent.clear()
    machine.move_to_position(x=5.0, z=-2.0, speed=1800)
    current = _decode_state(IDLE)
    offset = (0, 0, 0, 0) if slot == 0 else (11000, 22000, 33000, 44000)
    expected = (5000 + offset[0], round(current.y_mm * 1000),
                -2000 + offset[2], round(current.a_deg * 1000))
    assert (0x04f7, struct.pack('>HH4i', 1800, 0xffff, *expected)) in link.sent


def test_failed_coordinate_activation_does_not_substitute_machine_origin():
    machine = MDX40A(Link())
    machine._active_wcs = 1
    machine._wcs_offset = (10, 20, 30, 40)
    machine.get_wcs_origin = Mock(return_value=None)
    with pytest.raises(ValueError, match='previous coordinate system retained'):
        machine.set_active_wcs(3)
    assert machine.active_wcs == 1
    assert machine.wcs_offset == (10, 20, 30, 40)


def test_coordinate_menu_activation_then_move_menu_uses_same_origin():
    link = Link()
    machine = MDX40A(link)
    machine.get_wcs_origin = Mock(side_effect=lambda slot: (slot * 10.0, slot * 20.0, -slot * 5.0, 0.0))
    machine.poll = Mock(return_value=_decode_state(IDLE))
    tui = TUI(machine, _LogBuffer())
    tui._wcs_sel = 4
    tui._wcs_handle_key(10)
    assert machine.active_wcs == 4
    link.sent.clear()
    targets = dict(tui._move_targets())
    targets['XY Origin']()
    state = _decode_state(IDLE)
    assert (0x04f7, struct.pack('>HH4i', 3000, 0xffff, 40000, 80000,
                               round(state.z_mm * 1000), round(state.a_deg * 1000))) in link.sent
    machine._absolute_move = None
    link.sent.clear()
    targets['Preview / View Position (MCS)']()
    assert link.sent == [(0x1109, b'\0'), (0x0500, b'\xff\xff'), (0x1109, b'\xff')]


def test_absolute_move_keeps_operation_open_until_target_reached(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('rollingmill.machine.time.monotonic', lambda: clock[0])
    link = Link()
    machine = MDX40A(link)
    link.sent.clear()
    before = _decode_state(IDLE)
    target = (before.x_mm + 1.0, before.y_mm, before.z_mm, before.a_deg)
    machine.move_to_machine_pos(*target, speed=0xffff)
    assert [c for c, _ in link.sent] == [0x03f5, 0x1109, 0x04f7]
    assert link.sent[0] == (0x03f5, b'\x02')
    assert struct.unpack('>H', link.sent[-1][1][:2])[0] == 3000
    machine.poll()  # Fresh idle before motion acknowledgement.
    assert machine.move_active
    assert [c for c, _ in link.sent] == [0x03f5, 0x1109, 0x04f7]
    moving = bytearray(IDLE)
    moving[:4] = (before.flags | 0x04400000).to_bytes(4, 'big')
    link._responses[0x0100] = bytes(moving)
    clock[0] += 0.2
    machine.poll()
    assert machine.move_active
    # No idle heartbeat writes or premature 0x03f2/operation exit during motion.
    assert len(link.sent) == 3
    done = bytearray(IDLE)
    done[4:20] = struct.pack('>4i', *(round(v * 1000) for v in target))
    link._responses[0x0100] = bytes(done)
    machine.poll()
    assert not machine.move_active
    assert link.sent[-3:] == [(0x03f2, b''), (0x03f5, b'\xff'), (0x1109, b'\xff')]
    assert 'target position confirmed' in machine.move_message


def test_absolute_move_that_never_starts_reports_failure(monkeypatch):
    clock = [10.0]
    monkeypatch.setattr('rollingmill.machine.time.monotonic', lambda: clock[0])
    link = Link()
    machine = MDX40A(link)
    machine.move_to_machine_pos(0, 0, 0, 0)
    clock[0] = 12.1
    machine.poll()
    assert not machine.move_active
    assert 'did not start' in machine.move_message
    assert (0x03f3, b'') in link.sent


def test_absolute_move_blocks_keys_and_cancel_waits_for_idle():
    link = Link()
    machine = MDX40A(link)
    machine.move_to_machine_pos(0, 0, 0, 0)
    moving = bytearray(IDLE)
    moving[:4] = (int.from_bytes(moving[:4], 'big') | 0x04400000).to_bytes(4, 'big')
    link._responses[0x0100] = bytes(moving)
    tui = TUI(machine, _LogBuffer())
    count = len(link.sent)
    for key in ('z', 's', 'b', 'c', 'm'):
        tui._handle_key(ord(key))
    assert len(link.sent) == count
    tui._handle_key(27)
    assert (0x03f3, b'') in link.sent
    machine.poll()
    assert machine.move_active
    link._responses[0x0100] = IDLE
    machine.poll()
    assert not machine.move_active
    assert 'stopped' in machine.move_message
