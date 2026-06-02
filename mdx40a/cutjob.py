"""
CutJob — NC/RML file sender for the MDX desktop mills.

Parses a file into line-delimited blocks and drives transmission via
machine.bulk_write() and machine.get_nc_bytes_processed().

Single-threaded: the caller drives the state machine by calling service()
once per main-loop tick (~100 ms in the TUI). Typical use::

    job = CutJob.from_file(machine, "part.nc")
    job.start_step()          # arm; paused before block 0
    job.next_block()          # send block 0 (state → WAITING)
    # main loop calls job.service() each tick; WAITING polls the NC
    # bytes-processed counter and transitions back to STEPPING on ack.
    job.start_run()           # switch to continuous mode (auto next_block after each ack)

Wire model (RE'd from VP_MDX40A — see docs/sending-nc-code.md):
  - Both step and run mode send ONE block per USB bulk-OUT transfer.
  - After each block, poll GET 0x0200 until counter == before + len(block).
  - Run mode auto-advances on ack; step mode parks back in STEPPING.
  - There is no multi-block chunking on the wire — the firmware's internal
    NC buffer would overflow.

States:
  IDLE     — constructed but not started
  STEPPING — paused, waiting for next_block()
  WAITING  — step block sent, polling NC bytes-processed counter (→ STEPPING on ack)
  RUNNING  — run block sent, polling NC bytes-processed counter (→ next block on ack)
  DONE     — all blocks sent
  ERROR    — USB error or abort()
"""

import os
import time
from typing import List, Optional

from . import machine as _machine

def parse_nc_blocks(data: bytes) -> list:
    """Split NC/RML file bytes into delimiter-terminated blocks.

    Faithful to FUN_0040bc92 in VP_MDX40A.exe:
      - scans for CR or LF;
      - CRLF is consumed as a single delimiter (LF after CR is eaten);
      - each block is data[offsets[k]:offsets[k+1]] — i.e. includes its
        trailing delimiter byte(s) **as they appeared in the source**;
      - empty lines produce single-byte blocks (just the delimiter);
      - any trailing data after the final delimiter is the last block,
        with no synthetic delimiter appended.

    The counter at GET 0x0200 advances by exactly len(block), so any
    normalisation (e.g. promoting LF → CRLF) would break the expected-vs-
    actual counter math used by CutJob._service_ack.
    """
    offsets = [0]
    i, n = 0, len(data)
    while i < n:
        c = data[i]
        if c == 0x0D:                                # CR
            if i + 1 < n and data[i + 1] == 0x0A:    # CRLF — consume both
                i += 1
            offsets.append(i + 1)
        elif c == 0x0A:                              # bare LF
            offsets.append(i + 1)
        i += 1
    blocks = [data[offsets[k]:offsets[k+1]] for k in range(len(offsets) - 1)]
    if offsets[-1] < n:                              # tail with no trailing delimiter
        blocks.append(data[offsets[-1]:])
    return blocks


class CutJob:
    IDLE     = 'idle'
    STEPPING = 'step'
    WAITING  = 'wait'
    RUNNING  = 'run'
    DONE     = 'done'
    ERROR    = 'error'

    _WAIT_TIMEOUT = 60.0   # seconds to wait for NC counter to advance after a block

    # ── Construction ──────────────────────────────────────────────────────────

    def __init__(self, machine: _machine.MDX40A, blocks: List[bytes], filename: str = ''):
        self._m        = machine
        self._blocks   = blocks
        self._filename = filename
        self._idx      = 0
        self._state    = self.IDLE
        self._error: Optional[str] = None
        self._bracket_open = False

        # WAITING / RUNNING — set when a block is sent.
        self._wait_expected: int   = 0
        self._wait_deadline: float = 0.0

    @classmethod
    def from_bytes(cls, machine: _machine.MDX40A, data: bytes, filename: str = '') -> 'CutJob':
        """Parse *data* into blocks and return a CutJob ready to be started."""
        blocks = parse_nc_blocks(data)
        if not blocks:
            raise ValueError(f"No NC blocks found in payload ({len(data)} bytes)")
        return cls(machine, blocks, filename)

    @classmethod
    def from_file(cls, machine: _machine.MDX40A, path: str) -> 'CutJob':
        """Read *path*, parse into blocks, return a CutJob ready to be started."""
        with open(path, 'rb') as fh:
            data = fh.read()
        return cls.from_bytes(machine, data, os.path.basename(path))

    # ── Read-only status ──────────────────────────────────────────────────────

    @property
    def filename(self) -> str:
        return self._filename

    @property
    def state(self) -> str:
        return self._state

    @property
    def block_idx(self) -> int:
        return self._idx

    @property
    def total(self) -> int:
        return len(self._blocks)

    @property
    def error(self) -> Optional[str]:
        return self._error

    def line_at(self, idx: int) -> str:
        """Return the decoded text of block *idx*, or '' if out of range."""
        if 0 <= idx < len(self._blocks):
            return self._blocks[idx].rstrip(b'\r\n').decode('ascii', errors='replace')
        return ''

    # ── Control ───────────────────────────────────────────────────────────────

    def start_step(self) -> None:
        """Arm or switch to step mode (pause before each block).

        If a run-mode block is currently in flight (RUNNING), the in-flight
        block still completes; on its ack the job parks in STEPPING instead
        of auto-sending the next block.
        """
        if self._state in (self.DONE, self.ERROR):
            return
        if self._state == self.RUNNING:
            self._state = self.WAITING   # finish current block, then park.
        elif self._state != self.WAITING:
            self._state = self.STEPPING

    def start_run(self) -> None:
        """Switch to continuous mode: send one block per tick, gated by the
        NC bytes-processed counter (RE'd from VP_MDX40A nc_send_one_slice loop).
        """
        if self._state in (self.DONE, self.ERROR):
            return
        if self._idx >= len(self._blocks):
            self._state = self.DONE
            self._close_bracket()
            return
        if self._state == self.WAITING:
            # A step block is still acking — promote it so on ack we auto-advance.
            self._state = self.RUNNING
            return
        self._send_block(run_mode=True)

    def next_block(self) -> None:
        """Send the next block (step mode). STEPPING → WAITING."""
        if self._state != self.STEPPING:
            return
        if self._idx >= len(self._blocks):
            self._state = self.DONE
            self._close_bracket()
            return
        self._send_block(run_mode=False)

    def pause(self) -> None:
        """Pause run mode: stop auto-advancing after the in-flight block acks."""
        if self._state == self.RUNNING:
            self._state = self.WAITING   # on ack, _service_ack parks in STEPPING.

    def abort(self) -> None:
        """Stop immediately and enter ERROR state."""
        self._error = 'aborted'
        self._state = self.ERROR
        self._close_bracket()

    # ── Per-tick service ──────────────────────────────────────────────────────

    def service(self) -> None:
        """Advance the state machine by one step. Call once per main-loop tick."""
        if self._state == self.RUNNING:
            self._service_ack(run_mode=True)
        elif self._state == self.WAITING:
            self._service_ack(run_mode=False)
        # IDLE / STEPPING / DONE / ERROR: nothing to do

    def _service_ack(self, run_mode: bool) -> None:
        v = self._m.get_nc_bytes_processed()
        if v >= 0 and v == self._wait_expected:
            self._idx += 1
            if self._idx >= len(self._blocks):
                self._state = self.DONE
                self._close_bracket()
            elif run_mode:
                self._send_block(run_mode=True)   # auto-advance: stay in RUNNING
            else:
                self._state = self.STEPPING
            return
        # Counter not yet at expected — check if the firmware has flagged an
        # error (e.g. illegal block, wrong command-set mode). The error bit
        # (ping bit 20) stays set until power-cycled, so polling per-tick
        # surfaces the rejection at the offending block rather than after the
        # 10-second WAIT_TIMEOUT.
        if self._m.has_device_error():
            block_text = self.line_at(self._idx)
            self._set_error(
                f"firmware error bit set after block {self._idx} ({block_text!r}); "
                f"counter at {v}, expected {self._wait_expected}"
            )
            return
        if time.monotonic() > self._wait_deadline:
            self._set_error(
                f"timeout waiting for NC counter (have {v}, expected {self._wait_expected})"
            )

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _send_block(self, run_mode: bool) -> None:
        """Send self._blocks[self._idx] and enter WAITING/RUNNING.

        _idx stays put until _service_ack sees counter == expected; only then
        does it increment (matching VPanel's nc_send_one_slice + counter-poll loop).
        """
        block = self._blocks[self._idx]
        try:
            self._ensure_bracket()
            before = self._m.get_nc_bytes_processed()
            if before < 0:
                self._set_error("get_nc_bytes_processed failed before bulk_write")
                return
            self._m.bulk_write(block)
            self._wait_expected = (before + len(block)) & 0xFFFFFFFF
            self._wait_deadline = time.monotonic() + self._WAIT_TIMEOUT
            self._state         = self.RUNNING if run_mode else self.WAITING
        except Exception as exc:
            self._set_error(str(exc))

    def _ensure_bracket(self) -> None:
        if not self._bracket_open:
            self._m.begin_nc_job()
            self._bracket_open = True

    def _close_bracket(self) -> None:
        if self._bracket_open:
            self._m.end_nc_job()
            self._bracket_open = False

    def _set_error(self, msg: str) -> None:
        self._error = msg
        self._state = self.ERROR
        self._close_bracket()

