"""
CutJob — NC/RML file sender for the MDX desktop mills.

Parses a file into line-delimited blocks and drives transmission via
machine.bulk_write() and machine.get_nc_bytes_processed().

Single-threaded: the caller drives the state machine by calling service()
once per main-loop tick (~100 ms in the TUI). Typical use::

    job = CutJob.from_file(machine, "part.nc")
    job.step()                # send the next block (auto-arms STEP mode)
    # main loop calls job.service() each tick; it polls the NC bytes-
    # processed counter and, on ack, parks (STEP) or sends the next (RUN).
    job.run()                 # switch to continuous mode
    job.pause()               # stop after the in-flight block — park in STEP
    ...
    job.restart()             # after DONE/ERROR, rewind to block 0

UI code should not branch on `state` directly. Instead, use the
`can_run` / `can_step` / `can_pause` / `can_restart` predicates to gate
keys and to generate hint text — that keeps the state semantics inside
this module rather than scattered across callers.

State model:
  state ∈ {IDLE, STEP, RUN, DONE, ERROR}
    IDLE  — constructed but not started; no bracket open
    STEP  — interactive; one block per next_block()
    RUN   — continuous; service() auto-advances after each ack
    DONE  — all blocks sent; bracket closed
    ERROR — aborted or USB/firmware failure; bracket closed

  in_flight: bool — True between a block being sent and the firmware
    acknowledging it via the NC bytes-processed counter. Orthogonal to
    state — both STEP and RUN can be in flight.

Wire model (RE'd from VP_MDX40A — see docs/sending-nc-code.md):
  - Both step and run mode send ONE block per USB bulk-OUT transfer.
  - After each block, poll GET 0x0200 until counter == before + len(block).
  - In RUN mode the next block goes out as soon as the previous one acks.
  - There is no multi-block chunking on the wire — the firmware's internal
    NC buffer would overflow.
"""

import os
import time
from typing import Optional

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
    actual counter math used by CutJob.service.
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
    IDLE  = 'idle'
    STEP  = 'step'
    RUN   = 'run'
    DONE  = 'done'
    ERROR = 'error'

    _WAIT_TIMEOUT = 60.0   # seconds to wait for NC counter to advance after a block

    # ── Construction ──────────────────────────────────────────────────────────

    def __init__(self, machine: _machine.MDX40A, blocks: list[bytes], filename: str = ''):
        self._m        = machine
        self._blocks   = blocks
        self._filename = filename
        self._idx      = 0
        self._state    = self.IDLE
        self._error: Optional[str] = None
        self._bracket_open  = False
        self._in_flight     = False
        self._wait_expected = 0
        self._wait_deadline = 0.0

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

    @property
    def in_flight(self) -> bool:
        return self._in_flight

    # Action predicates — each one corresponds to exactly one control method.
    # UI code should ask these rather than inspecting `state` directly.

    @property
    def can_run(self) -> bool:
        """True iff calling run() would do something. False while RUN or after DONE/ERROR."""
        return self._state in (self.IDLE, self.STEP)

    @property
    def can_step(self) -> bool:
        """True iff calling step() would send a block. False while RUN, after
        DONE/ERROR, or while a block is already in flight."""
        return self._state in (self.IDLE, self.STEP) and not self._in_flight

    @property
    def can_pause(self) -> bool:
        """True iff calling pause() would stop a running job."""
        return self._state == self.RUN

    @property
    def can_restart(self) -> bool:
        """True iff calling restart() would rewind a finished or failed job."""
        return self._state in (self.DONE, self.ERROR)

    def line_at(self, idx: int) -> str:
        """Return the decoded text of block *idx*, or '' if out of range."""
        if 0 <= idx < len(self._blocks):
            return self._blocks[idx].rstrip(b'\r\n').decode('ascii', errors='replace')
        return ''

    # ── Control ───────────────────────────────────────────────────────────────
    #
    # Each method is a no-op when its corresponding `can_*` predicate is False,
    # so callers can fire them unconditionally if they prefer.

    def run(self) -> None:
        """Switch to run mode. If a block is in flight, the ack auto-advances
        to the next block; otherwise the next block is sent immediately."""
        if not self.can_run:
            return
        self._state = self.RUN
        if not self._in_flight:
            self._send_next_or_finish()

    def step(self) -> None:
        """Send the next block, then park in STEP mode after the ack.
        Idempotently switches to STEP from IDLE."""
        if not self.can_step:
            return
        self._state = self.STEP
        self._send_next_or_finish()

    def pause(self) -> None:
        """Stop a running job: the in-flight block completes, then parks in STEP."""
        if not self.can_pause:
            return
        self._state = self.STEP

    def restart(self) -> None:
        """Rewind to block 0 and return to IDLE. Only allowed when DONE/ERROR
        — restarting mid-job would desync the NC counter math."""
        if not self.can_restart:
            return
        self._idx       = 0
        self._error     = None
        self._in_flight = False
        self._state     = self.IDLE
        # Bracket was closed on entering DONE/ERROR; _send_next_or_finish reopens it.

    def abort(self) -> None:
        """Stop immediately and enter ERROR state. Always allowed."""
        self._error     = 'aborted'
        self._in_flight = False
        self._state     = self.ERROR
        self._close_bracket()

    # ── Per-tick service ──────────────────────────────────────────────────────

    def service(self) -> None:
        """Advance the state machine by one step. Call once per main-loop tick.

        With no block in flight, this is a no-op — the next outgoing block
        is triggered by next_block() (STEP) or by start_run() / by the
        ack-handler chaining the next send (RUN).
        """
        if not self._in_flight:
            return
        v = self._m.get_nc_bytes_processed()
        if v >= 0 and v == self._wait_expected:
            self._in_flight = False
            self._idx += 1
            if self._state == self.RUN:
                self._send_next_or_finish()
            elif self._idx >= len(self._blocks):
                self._state = self.DONE
                self._close_bracket()
            return
        # Counter not yet at expected — check if the firmware has flagged an
        # error (e.g. illegal block, wrong command-set mode). The error bit
        # (ping bit 20) stays set until power-cycled, so polling per-tick
        # surfaces the rejection at the offending block rather than after the
        # WAIT_TIMEOUT.
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

    def _send_next_or_finish(self) -> None:
        """Send self._blocks[self._idx]; or if no more blocks, transition to DONE."""
        if self._idx >= len(self._blocks):
            self._state = self.DONE
            self._close_bracket()
            return
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
            self._in_flight     = True
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
        self._error     = msg
        self._in_flight = False
        self._state     = self.ERROR
        self._close_bracket()
