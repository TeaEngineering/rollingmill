"""CutJob state-machine tests.

Drives `CutJob` over an `MdxMockUSB` subclass that fakes the NC bytes-processed
counter (GET 0x0200): `bulk_write` advances the counter by `len(data)` so the
next poll matches the expected value and the job acks immediately, mirroring
the firmware's per-block ack on real hardware.
"""

import pytest

from rollingmill.check import main as check_main
from rollingmill.cutjob import (
    CutJob,
    NCWarning,
    RULE_PAREN_EMBEDDED,
    RULE_PAREN_UNCLOSED,
    check_nc,
    parse_nc_blocks,
)
from rollingmill.machine import MDX40A
from rollingmill.usb import MdxMockUSB


class _AckingMockUSB(MdxMockUSB):
    """Mock that completes each bulk_write by advancing the NC counter."""

    def __init__(self) -> None:
        super().__init__()
        self._processed = 0

    def vend_get(self, wValue: int, length: int) -> bytes:
        if wValue == 0x0200 and length >= 4:
            return self._processed.to_bytes(4, 'big')
        return super().vend_get(wValue, length)

    def bulk_write(self, data: bytes) -> int:
        self._processed = (self._processed + len(data)) & 0xFFFFFFFF
        return len(data)


def _new_job(payload: bytes = b'G0X0\rG0Y0\rG0Z0\r') -> CutJob:
    m = MDX40A(_AckingMockUSB())
    return CutJob.from_bytes(m, payload, 'test.nc')


def test_step_from_idle_auto_arms_and_acks():
    job = _new_job()
    assert (job.state, job.block_idx, job.in_flight) == (CutJob.IDLE, 0, False)

    job.step()
    job.service()
    # After ack the job parks back in STEP with no flight, ready for the next step().
    assert (job.state, job.block_idx, job.in_flight) == (CutJob.STEP, 1, False)


def test_run_auto_advances_to_done():
    job = _new_job()
    job.run()
    for _ in range(10):
        job.service()
    assert (job.state, job.block_idx, job.in_flight) == (CutJob.DONE, job.total, False)


def test_restart_only_allowed_from_done_or_error():
    job = _new_job()

    # Mid-job: restart is rejected.
    job.step()
    job.restart()
    assert job.state == CutJob.STEP and job.block_idx == 0
    job.service()                                              # land the ack
    assert job.block_idx == 1

    # Drive to DONE, then restart.
    job.run()
    for _ in range(10):
        job.service()
    assert job.state == CutJob.DONE

    job.restart()
    assert (job.state, job.block_idx, job.in_flight, job.error) == \
           (CutJob.IDLE, 0, False, None)


def test_pause_during_run_preserves_in_flight():
    """`pause` flips mode to STEP but doesn't cancel the in-flight block — on
    the next ack the job parks in STEP rather than auto-advancing."""
    job = _new_job()
    job.run()
    assert (job.state, job.in_flight) == (CutJob.RUN, True)

    job.pause()
    assert (job.state, job.in_flight) == (CutJob.STEP, True)

    job.service()   # ack lands; STEP mode means we park rather than chain
    assert (job.state, job.block_idx, job.in_flight) == (CutJob.STEP, 1, False)


def test_restart_then_rerun_completes_again():
    """After a full DONE → restart cycle, the job can be run to completion again."""
    job = _new_job()
    job.run()
    for _ in range(10):
        job.service()
    assert job.state == CutJob.DONE

    job.restart()
    job.run()
    for _ in range(10):
        job.service()
    assert (job.state, job.block_idx) == (CutJob.DONE, job.total)


def test_can_predicates_track_state():
    """The can_* predicates are what the TUI uses to gate keys; this pins
    them across the typical lifecycle."""
    job = _new_job()
    # IDLE: can run or step; cannot pause or restart.
    assert (job.can_run, job.can_step, job.can_pause, job.can_restart) == \
           (True, True, False, False)

    # RUN with block in flight: only pause.
    job.run()
    assert (job.can_run, job.can_step, job.can_pause, job.can_restart) == \
           (False, False, True, False)

    # STEP in flight (we paused mid-run): can_run still allowed (resume);
    # can_step is gated by in_flight.
    job.pause()
    assert (job.can_run, job.can_step, job.can_pause, job.can_restart) == \
           (True, False, False, False)

    # STEP no flight (after ack): both r and x have meaning again.
    job.service()
    assert (job.can_run, job.can_step, job.can_pause, job.can_restart) == \
           (True, True, False, False)

    # DONE: only restart.
    job.run()
    for _ in range(10):
        job.service()
    assert (job.can_run, job.can_step, job.can_pause, job.can_restart) == \
           (False, False, False, True)


# ---- check_nc: comment-paren validation -----------------------------------

def test_parse_nc_blocks_does_not_validate():
    """parse_nc_blocks is a pure parser now — validation lives in check_nc."""
    # Would previously have raised; the parser should now return blocks
    # untouched even when a comment has embedded parens.
    blocks = parse_nc_blocks(b"(this (is) bad)\n")
    assert len(blocks) == 1


def test_check_nc_accepts_parse_nc_blocks_output_directly():
    """check_nc takes the raw bytes output of parse_nc_blocks; no decoding
    needed at the call site."""
    blocks = parse_nc_blocks(b"(hello)\nG01 X1.0\n(more)\n")
    assert list(check_nc(blocks)) == []


def test_check_nc_clean_file_returns_no_warnings():
    blocks = [b"(hello)\n", b"G01 X1.0\n", b"(more)\n"]
    assert list(check_nc(blocks)) == []


def test_check_nc_flags_embedded_close_paren():
    blocks = [b"G01 X1.0\n", b"(this (is) bad)\n", b"G01 Y2.0\n"]
    warnings = check_nc(blocks)
    assert len(warnings) == 1
    w = warnings[0]
    assert (w.rule, w.line) == (RULE_PAREN_EMBEDDED, 2)
    assert isinstance(w, NCWarning)


def test_check_nc_flags_embedded_open_paren():
    warnings = check_nc([b"(outer (inner)\n"])
    assert len(warnings) == 1
    assert warnings[0].rule == RULE_PAREN_EMBEDDED
    assert warnings[0].line == 1


def test_check_nc_flags_unclosed_comment():
    blocks = [b"G01 X1.0\n", b"G01 Y2.0\n", b"(forgot to close\n",
              b"G01 Z3.0\n"]
    warnings = check_nc(blocks)
    assert len(warnings) == 1
    assert warnings[0].rule == RULE_PAREN_UNCLOSED
    assert warnings[0].line == 3


def test_check_nc_ignores_parens_in_non_comment_lines():
    # A code line that happens to contain '(' or ')' mid-stream is NOT
    # subject to the rule -- only lines that *start* with '(' are
    # treated as comments by the firmware.
    assert list(check_nc([b"G01 X1.0 ( inline ) F100\n"])) == []


def test_check_nc_locates_column_of_first_embedded_paren():
    # '(ab)cd)' — first embedded ')' is at content[3] (0-based), so
    # 1-based column 4.
    warnings = check_nc([b"(ab)cd)\n"])
    assert len(warnings) == 1
    assert warnings[0].column == 4


def test_check_nc_accepts_empty_comment():
    assert list(check_nc([b"()\n"])) == []


def test_check_nc_reports_all_warnings_not_just_first():
    """The lint should surface every issue so the user can fix in one pass."""
    blocks = [b"(bad (one))\n", b"G01 X1.0\n", b"(unclosed\n"]
    warnings = check_nc(blocks)
    assert len(warnings) == 2
    rules = sorted(w.rule for w in warnings)
    assert rules == sorted([RULE_PAREN_EMBEDDED, RULE_PAREN_UNCLOSED])


# ---- from_bytes: gate the TUI streamer on check_nc ------------------------

def test_from_bytes_rejects_files_with_warnings():
    m = MDX40A(_AckingMockUSB())
    with pytest.raises(ValueError) as exc:
        CutJob.from_bytes(m, b"(bad (paren))\n", "x.nc")
    assert "rule 1" in str(exc.value)


# ---- main(): lint CLI -----------------------------------------------------

def test_main_clean_file_returns_zero(tmp_path, capsys):
    p = tmp_path / "clean.nc"
    p.write_bytes(b"(ok)\nG01 X1.0\n")
    rc = check_main([str(p)])
    out = capsys.readouterr().out
    assert rc == 0
    assert "OK" in out


def test_main_reports_warnings_in_three_line_format(tmp_path, capsys):
    """When validation fails, the CLI prints a one-line banner first, then
    three lines per warning (header, source, caret). capsys captures
    non-TTY stdout, so ANSI codes are absent."""
    p = tmp_path / "bad.nc"
    p.write_bytes(b"G01 X1.0\n(bad (paren))\n")
    rc = check_main([str(p)])
    captured = capsys.readouterr()
    assert rc == 1
    out_lines = captured.out.splitlines()
    # Banner + 3 lines per warning. Only one warning here.
    assert len(out_lines) == 4
    assert out_lines[0] == "Input file failed validation:"
    assert out_lines[1] == f"{p}:2:6: [rule 1] embedded parenthesis in comment"
    assert out_lines[2] == "(bad (paren))"
    # Column 6 → 5 leading spaces, then the caret under the embedded '('.
    assert out_lines[3] == "     ^"


def test_main_caret_under_unclosed_comment_is_column_one(tmp_path, capsys):
    p = tmp_path / "unclosed.nc"
    p.write_bytes(b"(forgot to close\n")
    rc = check_main([str(p)])
    out_lines = capsys.readouterr().out.splitlines()
    assert rc == 1
    # out_lines[0] is the banner; [1] header, [2] source, [3] caret.
    assert out_lines[0] == "Input file failed validation:"
    assert out_lines[2] == "(forgot to close"
    assert out_lines[3] == "^"


# ---- TUI: lint runs before USB --------------------------------------------

def test_tui_main_bails_on_bad_file_before_usb(tmp_path, monkeypatch, capsys):
    """Running ``python -m rollingmill.tui --file bad.nc`` should print the
    warnings and exit *before* MdxUSB.discover() is invoked — there's no
    point handshaking the mill only to abort after reading the file."""
    p = tmp_path / "bad.nc"
    p.write_bytes(b"G01 X1.0\n(bad (paren))\n")

    # Sentinel that fails the test if USB discovery is reached.
    from rollingmill import usb as usb_mod
    def _boom():
        raise AssertionError("MdxUSB.discover() was called before lint check")
    monkeypatch.setattr(usb_mod.MdxUSB, "discover", staticmethod(_boom))

    from rollingmill import tui as tui_mod
    rc = tui_mod.main(["--file", str(p)])
    assert rc == 1
    out = capsys.readouterr().out
    assert f"{p}:2:6:" in out
    assert "embedded parenthesis" in out


def test_main_usage_error_returns_two(capsys):
    rc = check_main([])
    assert rc == 2
    assert "usage" in capsys.readouterr().err.lower()


def test_main_missing_file_returns_two(tmp_path, capsys):
    rc = check_main([str(tmp_path / "nonexistent.nc")])
    assert rc == 2
    assert capsys.readouterr().err  # some error message printed
