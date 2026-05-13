"""
USB session tracer for Roland MDX-40A.

Each control transfer is written as a single line to a text file under
~/roland/YYYYMMDD-HHMMSS.sss.txt so sessions can be replayed or reviewed.

Line format (one transfer per line):
  HH:MM:SS.sss > SET wv=0x04f7 20: 03e8ffff00002af0...
  HH:MM:SS.sss < GET wv=0x0100 32: 0682081c000008ac...
  HH:MM:SS.sss ! ERR SET wv=0x0004  [Errno 32] Pipe error
  HH:MM:SS.sss # comment / marker

'>' = host→device (SET), '<' = device→host (GET), '!' = error, '#' = annotation.
"""

import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional


class Tracer:
    """Thread-safe USB transfer logger."""

    def __init__(self, path: Path):
        self._path = path
        self._lock = threading.Lock()
        self._fh = open(path, 'w', buffering=1)   # line-buffered
        self._start = time.monotonic()
        header = (
            f"# Roland MDX-40A USB trace\n"
            f"# Session start: {datetime.now().isoformat(timespec='milliseconds')}\n"
            f"# Format:  HH:MM:SS.sss DIR wv=0xNNNN [len: hexdata]\n"
            f"#   >  SET  host→device\n"
            f"#   <  GET  device→host\n"
            f"#   !  error\n"
            f"#   #  annotation\n"
            f"#\n"
        )
        self._fh.write(header)

    # ── Public API ────────────────────────────────────────────────────────────

    def log_set(self, wvalue: int, data: bytes) -> None:
        if data:
            payload = f"{len(data)}: {data.hex()}"
        else:
            payload = "0"
        self._write(f"> SET wv=0x{wvalue:04x} {payload}")

    def log_get(self, wvalue: int, data: bytes) -> None:
        self._write(f"< GET wv=0x{wvalue:04x} {len(data)}: {bytes(data).hex()}")

    def log_error(self, direction: str, wvalue: int, exc: Exception) -> None:
        self._write(f"! ERR {direction} wv=0x{wvalue:04x}  {exc}")

    def annotate(self, text: str) -> None:
        """Write a free-text marker line (e.g. key pressed, jog axis/dist)."""
        for line in text.splitlines():
            self._write(f"# {line}")

    def close(self) -> None:
        with self._lock:
            if not self._fh.closed:
                elapsed = time.monotonic() - self._start
                self._fh.write(f"# Session end  elapsed={elapsed:.3f}s\n")
                self._fh.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # ── Internal ──────────────────────────────────────────────────────────────

    def _write(self, text: str) -> None:
        ts = datetime.now().strftime('%H:%M:%S.') + f"{datetime.now().microsecond // 1000:03d}"
        with self._lock:
            self._fh.write(f"{ts} {text}\n")


def open_trace() -> Tracer:
    """Create ~/roland/<timestamp>.txt and return a Tracer for it."""
    directory = Path.home() / "roland"
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d-%H%M%S.') + f"{datetime.now().microsecond // 1000:03d}"
    path = directory / f"{stamp}.txt"
    return Tracer(path)


# ── Module-level active tracer (set by caller, used by usb.py) ───────────────

_active: Optional[Tracer] = None


def set_active(t: Optional['Tracer']) -> None:
    global _active
    _active = t


def get_active() -> Optional['Tracer']:
    return _active
