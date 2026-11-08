"""Z0 sensor thickness: GET 0x3900 (SET 0x3901 is the Setup dialog's write).

Decoded from VP_MDX40A.exe: the Setup dialog's "Thickness of Z0 Sensor:" edit box
(dialog 4050, control 0x0FD9) is loaded from GET 0x3900 and saved with SET 0x3901,
both as a big-endian uint32 in 1/1000 mm.
"""
import struct

import pytest
import usb.core

from rollingmill import decode
from rollingmill.machine import MDX40A
from rollingmill.usb import MdxMockUSB


def test_get_z0_sensor_thickness_reads_micrometres_as_mm():
    mock = MdxMockUSB(responses={0x3900: struct.pack('>I', 15000)})
    assert MDX40A(mock).get_z0_sensor_thickness_mm() == pytest.approx(15.0)


def test_thickness_is_cached_at_connect():
    mock = MdxMockUSB(responses={0x3900: struct.pack('>I', 15100)})
    assert MDX40A(mock).z0_sensor_thickness_mm == pytest.approx(15.1)


def test_thickness_read_failure_is_none():
    class _Failing(MdxMockUSB):
        def vend_get(self, wValue, length):
            if wValue == 0x0001:
                raise usb.core.USBError("boom")
            return super().vend_get(wValue, length)

    m = MDX40A(_Failing())
    assert m.get_z0_sensor_thickness_mm() is None
    assert m.z0_sensor_thickness_mm is None


def test_decode_labels_read_and_write_as_thickness():
    read = decode.decode_line("t", '<', 0x0003, struct.pack('>I', 15000), last_trigger=0x3900)
    assert "Z0_SENSOR_THICKNESS" in read and "15.000 mm" in read
    write = decode.decode_line("t", '>', 0x3901, struct.pack('>I', 14500))
    assert "Z0_SENSOR_THICKNESS_WRITE" in write and "14.500 mm" in write
    assert "RPM" not in read + write


# ── setter and the 'b' dialog ────────────────────────────────────────────────

class _Recording(MdxMockUSB):
    """Mock whose GET 0x3900 follows SET 0x3901, like the firmware."""

    def __init__(self, thickness_um=14140, accept=True):
        super().__init__(responses={0x3900: struct.pack('>I', thickness_um)})
        self.sent = []
        self.accept = accept

    def vend_set(self, wValue, data=b""):
        self.sent.append((wValue, data))
        if wValue == 0x3901 and self.accept:
            self._responses[0x3900] = data
        super().vend_set(wValue, data)


def test_set_writes_bracketed_uint32_and_verifies_readback():
    link = _Recording()
    m = MDX40A(link)
    link.sent.clear()
    assert m.set_z0_sensor_thickness_mm(15.03) is True
    assert link.sent == [(0x1109, b'\x00'), (0x3901, struct.pack('>I', 15030)), (0x1109, b'\xff'),
                         (0x3900, b'')]
    assert m.z0_sensor_thickness_mm == pytest.approx(15.03)


@pytest.mark.parametrize("bad", [13.999, 16.001, 0.0, -1.0, 150.0])
def test_set_rejects_values_outside_vpanel_range_without_writing(bad):
    link = _Recording()
    m = MDX40A(link)
    link.sent.clear()
    with pytest.raises(ValueError):
        m.set_z0_sensor_thickness_mm(bad)
    assert link.sent == []


def test_set_reports_failure_when_readback_differs():
    link = _Recording(accept=False)
    m = MDX40A(link)
    assert m.set_z0_sensor_thickness_mm(15.03) is False
    assert m.z0_sensor_thickness_mm == pytest.approx(14.14)  # cache shows what the machine holds


def _type(tui, text):
    for ch in text:
        tui._handle_key(ord(ch))


def test_dialog_shows_stored_value_and_h_sets_it():
    from rollingmill.tui import TUI, _LogBuffer
    link = _Recording()
    m = MDX40A(link)
    tui = TUI(m, _LogBuffer())
    tui._handle_key(ord('b'))
    assert tui._probe_confirm and tui._probe_buf == '14.140' and not tui._probe_dirty
    link.sent.clear()
    _type(tui, '15.03')
    assert tui._probe_buf == '15.03' and tui._probe_dirty
    assert link.sent == []                       # typing alone writes nothing
    tui._handle_key(ord('h'))
    assert (0x3901, struct.pack('>I', 15030)) in link.sent
    assert not tui._probe_dirty and tui._probe_buf == '15.030'
    assert tui._probe_confirm                    # dialog stays open after H


def test_enter_is_refused_until_edited_value_is_set():
    from rollingmill.tui import TUI, _LogBuffer
    link = _Recording()
    m = MDX40A(link)
    m.set_active_wcs(1)
    tui = TUI(m, _LogBuffer())
    tui._handle_key(ord('b'))
    _type(tui, '15.03')
    link.sent.clear()
    tui._handle_key(10)
    assert tui._probe_confirm and not tui._probe.active
    assert "H" in tui._probe_notice and link.sent == []


def test_enter_is_refused_for_a_workspace_without_a_sensor_cycle():
    from rollingmill.tui import TUI, _LogBuffer
    link = _Recording()
    tui = TUI(MDX40A(link), _LogBuffer())            # MCS is active
    tui._handle_key(ord('b'))
    link.sent.clear()
    tui._handle_key(10)
    assert tui._probe_confirm and not tui._probe.active
    assert "Select User" in tui._probe_notice and link.sent == []


@pytest.mark.parametrize("config,expected", [(b"\x00\x00", 0), (b"\x00\x01", 1), (b"\x00\x02", 2)])
def test_dialog_reads_command_set_when_opened(config, expected):
    from rollingmill.tui import TUI, _LogBuffer
    link = _Recording()
    link._responses[0x3106] = config + b"\x00" * 4
    tui = TUI(MDX40A(link), _LogBuffer())
    tui._handle_key(ord('b'))
    assert tui._probe_cmdset == expected


def test_out_of_range_h_keeps_dialog_and_writes_nothing():
    from rollingmill.tui import TUI, _LogBuffer
    link = _Recording()
    tui = TUI(MDX40A(link), _LogBuffer())
    tui._handle_key(ord('b'))
    _type(tui, '20')
    link.sent.clear()
    tui._handle_key(ord('h'))
    assert link.sent == [] and tui._probe_dirty and "14.000" in tui._probe_notice


def test_esc_closes_without_writing():
    from rollingmill.tui import TUI, _LogBuffer
    link = _Recording()
    tui = TUI(MDX40A(link), _LogBuffer())
    tui._handle_key(ord('b'))
    _type(tui, '15')
    link.sent.clear()
    tui._handle_key(27)
    assert not tui._probe_confirm and link.sent == []
