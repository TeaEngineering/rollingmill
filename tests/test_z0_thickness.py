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
