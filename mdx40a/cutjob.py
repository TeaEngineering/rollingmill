"""
CutJob — NC/RML file sender for the Roland MDX-40A.

Parses a file into line-delimited blocks and drives transmission via
MDX40A.bulk_write() and MDX40A.get_nc_bytes_processed().

Typical use::

    job = CutJob.from_file(machine, "part.nc")
    job.start_step()          # arm; paused before block 0
    job.next_block()          # send block 0, wait for counter ack
    job.start_run()           # switch to continuous bulk streaming

States:
  IDLE     — constructed but not started
  STEPPING — paused before next block, waiting for next_block()
  WAITING  — block sent, polling NC bytes-processed counter
  RUNNING  — streaming file in 32 KB bulk chunks
  DONE     — all blocks sent
  ERROR    — USB error or abort()
"""

import os
import threading
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

    _CHUNK = 0x8000   # 32 KB — matches VPanel CFile::Read buffer

    # ── Construction ──────────────────────────────────────────────────────────

    def __init__(self, machine: _machine.MDX40A, blocks: List[bytes], filename: str = ''):
        self._m        = machine
        self._blocks   = blocks
        self._filename = filename
        self._idx      = 0
        self._state    = self.IDLE
        self._lock     = threading.Lock()
        self._next     = threading.Event()   # fired by next_block() / start_run()
        self._error: Optional[str] = None
        self._thr: Optional[threading.Thread] = None

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
        with self._lock:
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
        with self._lock:
            if self._state in (self.DONE, self.ERROR):
                return
            self._state = self.STEPPING
        self._ensure_thread()

    def start_run(self) -> None:
        """Switch to continuous bulk streaming."""
        with self._lock:
            if self._state in (self.DONE, self.ERROR):
                return
            self._state = self.RUNNING
        self._next.set()
        self._ensure_thread()

    def next_block(self) -> None:
        """Send the next block (step mode)."""
        self._next.set()

    def pause(self) -> None:
        """Switch from run to step mode after the current chunk finishes."""
        with self._lock:
            if self._state == self.RUNNING:
                self._state = self.STEPPING

    def abort(self) -> None:
        """Stop immediately and enter ERROR state."""
        with self._lock:
            self._state = self.ERROR
            self._error = 'aborted'
        self._next.set()

    # ── Background thread ─────────────────────────────────────────────────────

    def _ensure_thread(self) -> None:
        if self._thr is None or not self._thr.is_alive():
            self._thr = threading.Thread(target=self._loop, daemon=True, name='cut')
            self._thr.start()

    def _loop(self) -> None:
        while True:
            with self._lock:
                st  = self._state
                idx = self._idx
            if st in (self.DONE, self.ERROR):
                return
            if idx >= len(self._blocks):
                with self._lock:
                    self._state = self.DONE
                return
            if st == self.RUNNING:
                self._run_bulk(idx)
                return
            if st == self.STEPPING:
                self._step_one(idx)
            else:
                time.sleep(0.05)

    def _run_bulk(self, start: int) -> None:
        payload = b''.join(self._blocks[start:])
        offset  = 0
        try:
            while offset < len(payload):
                with self._lock:
                    st = self._state
                if st == self.ERROR:
                    return
                if st == self.STEPPING:
                    # switched to pause mid-stream — find the enclosing block
                    acc = 0
                    for i, blk in enumerate(self._blocks[start:]):
                        acc += len(blk)
                        if acc > offset:
                            with self._lock:
                                self._idx = start + i
                            break
                    self._loop()
                    return
                chunk = payload[offset:offset + self._CHUNK]
                self._m.bulk_write(chunk)
                sent = offset + len(chunk)
                acc  = 0
                for i, blk in enumerate(self._blocks[start:]):
                    acc += len(blk)
                    if acc >= sent:
                        with self._lock:
                            self._idx = start + i + 1
                        break
                offset += len(chunk)
        except Exception as exc:
            with self._lock:
                self._state = self.ERROR
                self._error = str(exc)
            return
        with self._lock:
            self._idx   = len(self._blocks)
            self._state = self.DONE

    def _step_one(self, idx: int) -> None:
        self._next.clear()
        self._next.wait()
        self._next.clear()
        with self._lock:
            st = self._state
        if st == self.ERROR:
            return
        if st == self.RUNNING:
            self._loop()
            return
        with self._lock:
            self._state = self.WAITING
        block = self._blocks[idx]
        try:
            before = self._m.get_nc_bytes_processed()
            self._m.bulk_write(block)
            if before >= 0:
                expected = (before + len(block)) & 0xFFFFFFFF
                deadline = time.monotonic() + 10.0
                while time.monotonic() < deadline:
                    with self._lock:
                        if self._state == self.ERROR:
                            return
                    v = self._m.get_nc_bytes_processed()
                    if v >= 0 and v == expected:
                        break
                    time.sleep(0.020)
        except Exception as exc:
            with self._lock:
                self._state = self.ERROR
                self._error = str(exc)
            return
        with self._lock:
            self._idx = idx + 1
            if self._state not in (self.ERROR, self.RUNNING):
                self._state = self.STEPPING
