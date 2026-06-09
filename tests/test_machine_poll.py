"""MDX40A.poll() decodes XYZA position from a faked state block.

Drives the machine against `MdxMockUSB` with a fake 0x0100 state response, then
checks the decoded `MachineState`.
"""
import struct

import pytest

from rollingmill.machine import FLAG_NC_READY, MDX40A
from rollingmill.usb import MdxMockUSB


def _state_block(
    flags: int,
    x_mm: float, y_mm: float, z_mm: float, a_deg: float,
    spindle_rpm: int = 0,
) -> bytes:
    """Pack a state block matching `_decode_state` (>I 4i I, big-endian)."""
    return struct.pack(
        '>I4iI',
        flags,
        round(x_mm * 1000),
        round(y_mm * 1000),
        round(z_mm * 1000),
        round(a_deg * 1000),
        spindle_rpm,
    )


def test_poll_decodes_xyza_position_from_mock():
    # NC_READY (bit 23) + STATE=2 (bits 18-16) → idle firmware
    flags = FLAG_NC_READY | (2 << 16)
    mock = MdxMockUSB(responses={
        0x0100: _state_block(flags, 12.345, -67.891, 5.000, 90.500, 4500),
    })

    m = MDX40A(mock)
    s = m.poll()

    assert s is not None
    assert s.flags == flags
    assert s.idle
    assert s.x_mm == pytest.approx(12.345)
    assert s.y_mm == pytest.approx(-67.891)
    assert s.z_mm == pytest.approx(5.000)
    assert s.a_deg == pytest.approx(90.500)
    assert s.spindle_rpm == 4500

    # Cached on the machine instance too.
    assert m.state.x_mm == pytest.approx(12.345)
    assert m.state.a_deg == pytest.approx(90.500)
