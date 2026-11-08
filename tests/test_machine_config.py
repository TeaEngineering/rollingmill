"""Machine-config struct (GET 0x3106 / SET 0x3107): command set + NC Code preferences.

Decoded from VP_MDX40A.exe: byte 1 is the command set, bytes 2-4 are the radio groups of
the "NC Code setting" page (decimal method, tool-diameter offset type, optional block),
bytes 0 and 5 are only round-tripped. See machine.MachineConfig.
"""
import logging

import pytest
import usb.core

from rollingmill import machine as M
from rollingmill.machine import MDX40A, MachineConfig
from rollingmill.usb import MdxMockUSB


class _Config(MdxMockUSB):
    """Mock whose GET 0x3106 follows SET 0x3107, like the firmware (unless it refuses writes)."""

    def __init__(self, raw=bytes([0, 1, 0, 0, 0, 5]), accept=True, fail_write=False):
        super().__init__(responses={0x3106: raw})
        self.sent = []
        self.accept = accept
        self.fail_write = fail_write

    def vend_set(self, wValue, data=b""):
        self.sent.append((wValue, data))
        if wValue == 0x3107:
            if self.fail_write:
                raise usb.core.USBError("boom")
            if self.accept:
                self._responses[0x3106] = data
        super().vend_set(wValue, data)

    def writes(self):
        return [d for w, d in self.sent if w == 0x3107]


def test_connect_reads_and_decodes_the_struct():
    m = MDX40A(_Config(bytes([9, 2, 3, 1, 1, 7])))
    cfg = m.config
    assert cfg.raw == bytes([9, 2, 3, 1, 1, 7])
    assert (cfg.command_set, cfg.decimal_method, cfg.offset_type, cfg.optional_block) == (2, 3, 1, 1)
    assert m.command_set == M.CMDSET_AUTO


def test_failed_read_at_connect_leaves_the_cache_empty():
    class _Failing(_Config):
        def vend_get(self, wValue, length):
            if wValue == 0x0001:
                raise usb.core.USBError("boom")
            return super().vend_get(wValue, length)

    m = MDX40A(_Failing())
    assert m.config is None and m.command_set is None
    assert m.query_command_set() is None


def test_write_config_sends_the_whole_struct_changing_only_the_edited_bytes():
    link = _Config(bytes([0, 1, 0, 0, 0, 5]))
    m = MDX40A(link)
    assert m.write_config(offset_type=M.OFFSET_TYPE_B, decimal_method=M.DECIMAL_CALC_F,
                          optional_block=M.OPTIONAL_BLOCK_DISABLED) is True
    # byte 2 decimal, byte 3 offset type, byte 4 optional block; bytes 0, 1 and 5 untouched
    assert link.writes() == [bytes([0, 1, 2, 1, 1, 5])]
    assert m.config.raw == bytes([0, 1, 2, 1, 1, 5])


def test_write_config_is_a_single_write_for_several_fields():
    link = _Config()
    m = MDX40A(link)
    m.write_config(command_set=M.CMDSET_NC, offset_type=M.OFFSET_TYPE_B)
    assert len(link.writes()) == 1


def test_unchanged_config_is_not_written():
    link = _Config(bytes([0, 2, 1, 1, 0, 5]))
    m = MDX40A(link)
    assert m.write_config(command_set=2, decimal_method=1, offset_type=1, optional_block=0) is True
    assert m.write_config() is True
    assert link.writes() == []


def test_set_command_set_only_changes_byte_1():
    link = _Config(bytes([0, 0, 3, 1, 1, 5]))
    m = MDX40A(link)
    assert m.set_command_set(M.CMDSET_NC) is True
    assert link.writes() == [bytes([0, 1, 3, 1, 1, 5])]
    assert m.command_set == M.CMDSET_NC


@pytest.mark.parametrize("kwargs", [dict(command_set=3), dict(command_set=-1), dict(decimal_method=4),
                                    dict(offset_type=2), dict(optional_block=2)])
def test_invalid_values_raise_before_any_write(kwargs):
    link = _Config()
    m = MDX40A(link)
    with pytest.raises(ValueError):
        m.write_config(**kwargs)
    assert link.writes() == []


def test_set_command_set_rejects_an_invalid_mode():
    link = _Config()
    with pytest.raises(ValueError):
        MDX40A(link).set_command_set(3)
    assert link.writes() == []


def test_a_write_the_machine_does_not_keep_is_reported_as_failed():
    link = _Config(bytes([0, 1, 0, 0, 0, 5]), accept=False)
    m = MDX40A(link)
    assert m.write_config(offset_type=M.OFFSET_TYPE_B) is False
    assert m.config.offset_type == M.OFFSET_TYPE_A    # the cache shows what the machine holds


def test_usb_failure_on_write_is_reported_as_failed():
    m = MDX40A(_Config(fail_write=True))
    assert m.write_config(command_set=M.CMDSET_AUTO) is False


def test_with_changes_leaves_other_bytes_alone():
    cfg = MachineConfig(bytes([7, 1, 2, 1, 0, 9]))
    assert cfg.with_changes(optional_block=1).raw == bytes([7, 1, 2, 1, 1, 9])
    assert cfg.with_changes().raw == cfg.raw


# ── labels and the startup banner ────────────────────────────────────────────

def test_offset_type_is_labelled_type_a_and_type_b():
    assert M.OFFSET_TYPE_LABELS == {M.OFFSET_TYPE_A: "Type A", M.OFFSET_TYPE_B: "Type B"}


def test_summary_names_every_field_and_shows_the_raw_bytes():
    cfg = MachineConfig(bytes([0, 1, 2, 1, 1, 5]))
    assert cfg.summary() == ("command set NC Code, tool-dia offset paths Type B, "
                             "decimal point Calculator Method [F], optional block Disabled "
                             "[00 01 02 01 01 05]")


def test_summary_survives_values_the_dialog_does_not_know():
    text = MachineConfig(bytes([0, 9, 7, 3, 5, 255])).summary()
    assert "command set unknown (9)" in text and "decimal point unknown (7)" in text
    assert "[00 09 07 03 05 ff]" in text


def test_startup_banner_prints_the_whole_config(caplog):
    with caplog.at_level(logging.INFO, logger="rollingmill.machine"):
        MDX40A(_Config(bytes([0, 2, 1, 0, 0, 5])))
    banner = [r.getMessage() for r in caplog.records if r.getMessage().startswith("Machine config:")]
    assert banner == ["Machine config: command set Selected automatically (RML-1/NC Code), "
                      "tool-dia offset paths Type A, decimal point Calculator Method [All], "
                      "optional block Enabled [00 02 01 00 00 05]"]


def test_startup_banner_when_the_config_cannot_be_read(caplog):
    class _Failing(_Config):
        def vend_get(self, wValue, length):
            if wValue == 0x0001:
                raise usb.core.USBError("boom")
            return super().vend_get(wValue, length)

    with caplog.at_level(logging.INFO, logger="rollingmill.machine"):
        MDX40A(_Failing())
    assert any(r.getMessage() == "Machine config read failed" for r in caplog.records)
