"""MDX40A.poll() rising-edge detection for the firmware error flag.

Drives the machine over a mock that can flip ping bit 20 (the firmware
error flag) and stage a code at the GET 0x2100 register. Each test uses
pytest's `caplog` fixture to assert what was logged.
"""

import logging
import struct

from rollingmill.machine import MDX40A, MDX_ERROR_MESSAGES
from rollingmill.usb import MdxMockUSB


class _ErrorMockUSB(MdxMockUSB):
    """Mock that lets a test set the firmware error flag and the code returned
    by the error-status register at GET 0x2100. The ping word keeps byte[3]=0xff
    so Pattern B reads still see 'response ready' regardless of the error bit."""

    def __init__(self, error_bit: bool = False, error_code: int = 0x107) -> None:
        super().__init__()
        self.error_bit = error_bit
        self.error_code = error_code

    def vend_get(self, wValue: int, length: int) -> bytes:
        if wValue == 0x0001 and length >= 4:
            # byte[2] bit 4 = ping bit 20 = _PING_ERROR_MASK.
            # byte[3] = 0xff so pattern_b_read sees "max response length ready".
            return bytes([0, 0, 0x10 if self.error_bit else 0, 0xff])
        if wValue == 0x0003 and self._last_set_wvalue == 0x2100:
            # 5 × BE uint16: VPanel reads local_c[1] (the second one).
            return struct.pack('>HH', 0, self.error_code) + b'\x00' * 6
        return super().vend_get(wValue, length)


def _warnings(caplog) -> list:
    return [r.message for r in caplog.records if r.levelno == logging.WARNING]


def test_poll_logs_error_message_on_rising_edge(caplog):
    """Rising edge of ping bit 20 fires exactly one WARNING per transition,
    carrying the decoded message and the hex code."""
    mock = _ErrorMockUSB()
    m = MDX40A(mock)

    with caplog.at_level(logging.WARNING, logger='mdx40a.machine'):
        # No-error polls are silent.
        m.poll()
        m.poll()
        assert _warnings(caplog) == []

        # Rising edge → one WARNING with the decoded message + hex code.
        mock.error_bit = True
        mock.error_code = 0x107
        m.poll()
        warns = _warnings(caplog)
        assert len(warns) == 1
        assert MDX_ERROR_MESSAGES[0x107] in warns[0]
        assert '0x0107' in warns[0]

        # Steady error → no further logs (the firmware holds the bit until
        # power-cycle; we only want one entry per transition).
        m.poll()
        m.poll()
        assert len(_warnings(caplog)) == 1

        # Clear bit, then re-raise with a different code → second WARNING.
        mock.error_bit = False
        m.poll()
        mock.error_bit = True
        mock.error_code = 0x307
        m.poll()
        warns = _warnings(caplog)
        assert len(warns) == 2
        assert MDX_ERROR_MESSAGES[0x307] in warns[1]
        assert '0x0307' in warns[1]


def test_error_message_appends_hex_for_known_code():
    s = MDX40A.error_message(0x107)
    assert s == f"{MDX_ERROR_MESSAGES[0x107]} (0x0107)"


def test_error_message_falls_back_for_unknown_code():
    s = MDX40A.error_message(0x999)
    # Generic prefix, but the raw hex is still visible.
    assert 'Unknown error' in s
    assert '0x0999' in s


def test_error_message_handles_none():
    # None reflects a read failure — no hex to show, but the caller still
    # gets a non-empty string to put in a log line.
    s = MDX40A.error_message(None)
    assert 'Unknown error' in s
    assert 'no code read' in s
