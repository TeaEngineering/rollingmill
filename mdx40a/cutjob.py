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
    job.start_run()           # switch to continuous bulk streaming

States:
  IDLE     — constructed but not started
  STEPPING — paused, waiting for next_block()
  WAITING  — block sent, polling NC bytes-processed counter
  RUNNING  — streaming file in 32 KB bulk chunks
  DONE     — all blocks sent
  ERROR    — USB error or abort()
"""

import os
import time
from typing import List, Optional

from . import machine as _machine


def parse_nc_blocks(data: bytes) -> list:
    """Split NC/RML file bytes into \\r\\n-terminated line blocks.

    Mirrors FUN_0040bc92 in VP_MDX40A.exe: scans for CR/LF delimiters,
    skips empty lines, appends \\r\\n to each block. Returns list[bytes].
    """
    blocks = []
    i = start = 0
    while i < len(data):
        c = data[i]
        if c in (0x0D, 0x0A):
            line = data[start:i]
            if line.strip():
                blocks.append(line + b'\r\n')
            if c == 0x0D and i + 1 < len(data) and data[i + 1] == 0x0A:
                i += 1
            start = i + 1
        i += 1
    tail = data[start:]
    if tail.strip():
        blocks.append(tail if tail.endswith(b'\r\n') else tail + b'\r\n')
    return blocks


class CutJob:
    IDLE     = 'idle'
    STEPPING = 'step'
    WAITING  = 'wait'
    RUNNING  = 'run'
    DONE     = 'done'
    ERROR    = 'error'

    _CHUNK = 0x8000        # 32 KB — matches VPanel CFile::Read buffer
    _WAIT_TIMEOUT = 10.0   # seconds to wait for NC counter to advance after a step block

    # ── Construction ──────────────────────────────────────────────────────────

    def __init__(self, machine: _machine.MDX40A, blocks: List[bytes], filename: str = ''):
        self._m        = machine
        self._blocks   = blocks
        self._filename = filename
        self._idx      = 0
        self._state    = self.IDLE
        self._error: Optional[str] = None
        self._bracket_open = False

        # WAITING — set when a step block is sent.
        self._wait_expected: int   = 0
        self._wait_deadline: float = 0.0

        # RUNNING — joined remaining-block payload + write offset.
        self._payload: bytes           = b''
        self._payload_start_idx: int   = 0
        self._offset: int              = 0

    @classmethod
    def from_file(cls, machine: _machine.MDX40A, path: str) -> 'CutJob':
        """Read *path*, parse into blocks, return a CutJob ready to be started."""
        with open(path, 'rb') as fh:
            data = fh.read()
        blocks = parse_nc_blocks(data)
        if not blocks:
            raise ValueError(f"No NC blocks found in {path!r}")
        return cls(machine, blocks, os.path.basename(path))

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
        """Arm or switch to step mode (pause before each block)."""
        if self._state in (self.DONE, self.ERROR):
            return
        if self._state == self.RUNNING:
            self._idx = self._idx_from_offset()
        self._state = self.STEPPING

    def start_run(self) -> None:
        """Switch to continuous bulk streaming from the current block."""
        if self._state in (self.DONE, self.ERROR):
            return
        if self._idx >= len(self._blocks):
            self._state = self.DONE
            self._close_bracket()
            return
        self._ensure_bracket()
        self._payload           = b''.join(self._blocks[self._idx:])
        self._payload_start_idx = self._idx
        self._offset            = 0
        self._state             = self.RUNNING

    def next_block(self) -> None:
        """Send the next block (step mode). STEPPING → WAITING."""
        if self._state != self.STEPPING:
            return
        if self._idx >= len(self._blocks):
            self._state = self.DONE
            self._close_bracket()
            return
        block = self._blocks[self._idx]
        try:
            self._ensure_bracket()
            before = self._m.get_nc_bytes_processed()
            self._m.bulk_write(block)
            if before >= 0:
                self._wait_expected = (before + len(block)) & 0xFFFFFFFF
                self._wait_deadline = time.monotonic() + self._WAIT_TIMEOUT
                self._state         = self.WAITING
            else:
                # Counter read failed (e.g. mock mode); skip the wait and advance.
                self._idx += 1
                if self._idx >= len(self._blocks):
                    self._state = self.DONE
                    self._close_bracket()
        except Exception as exc:
            self._set_error(str(exc))

    def pause(self) -> None:
        """Switch from run to step mode at the next chunk boundary."""
        if self._state == self.RUNNING:
            self._idx   = self._idx_from_offset()
            self._state = self.STEPPING

    def abort(self) -> None:
        """Stop immediately and enter ERROR state."""
        self._error = 'aborted'
        self._state = self.ERROR
        self._close_bracket()

    # ── Per-tick service ──────────────────────────────────────────────────────

    def service(self) -> None:
        """Advance the state machine by one step. Call once per main-loop tick."""
        st = self._state
        if st == self.RUNNING:
            self._service_running()
        elif st == self.WAITING:
            self._service_waiting()
        # IDLE / STEPPING / DONE / ERROR: nothing to do

    def _service_running(self) -> None:
        if self._offset >= len(self._payload):
            self._idx   = len(self._blocks)
            self._state = self.DONE
            self._close_bracket()
            return
        try:
            chunk = self._payload[self._offset:self._offset + self._CHUNK]
            self._m.bulk_write(chunk)
            self._offset += len(chunk)
            self._idx     = self._idx_from_offset()
            if self._offset >= len(self._payload):
                self._idx   = len(self._blocks)
                self._state = self.DONE
                self._close_bracket()
        except Exception as exc:
            self._set_error(str(exc))

    def _service_waiting(self) -> None:
        v = self._m.get_nc_bytes_processed()
        if v >= 0 and v == self._wait_expected:
            self._idx += 1
            if self._idx >= len(self._blocks):
                self._state = self.DONE
                self._close_bracket()
            else:
                self._state = self.STEPPING
            return
        if time.monotonic() > self._wait_deadline:
            self._set_error(
                f"timeout waiting for NC counter (have {v}, expected {self._wait_expected})"
            )

    # ── Internal helpers ──────────────────────────────────────────────────────

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

    def _idx_from_offset(self) -> int:
        """Index of the first block whose end is past the current write offset.

        Used when pausing mid-stream so next_block() resumes at the block
        enclosing the current offset (matching VPanel's behaviour).
        """
        acc = 0
        for i, blk in enumerate(self._blocks[self._payload_start_idx:]):
            acc += len(blk)
            if acc > self._offset:
                return self._payload_start_idx + i
        return len(self._blocks)
