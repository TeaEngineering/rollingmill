"""
USB session tracer for MDX desktop mills.

Each control transfer is written as a single line to a text file under
~/roland/YYYYMMDD-HHMMSS.sss.txt so sessions can be replayed or reviewed.

Line format (one transfer per line):
  HH:MM:SS.sss > SET wv=0x04f7 20: 03e8ffff00002af0...
  HH:MM:SS.sss < GET wv=0x0100 32: 0682081c000008ac...
  HH:MM:SS.sss > BULK 32: 5047...                # raw NC code on bulk-OUT endpoint
  HH:MM:SS.sss ! ERR SET wv=0x0004  [Errno 32] Pipe error
  HH:MM:SS.sss # comment / marker

'>' = host→device (SET/BULK), '<' = device→host (GET), '!' = error, '#' = annotation.
"""

import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional
from types import TracebackType

from .usb import MdxLink


class Tracer:
    """Thread-safe USB transfer logger."""

    def __init__(self, path: Path):
        self._path = path
        self._lock = threading.Lock()
        self._fh = open(path, "w", buffering=1)  # line-buffered
        self._start = time.monotonic()
        header = (
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

    def log_bulk(self, data: bytes) -> None:
        """Log a bulk-OUT transfer (raw NC code stream)."""
        self._write(f"> BULK {len(data)}: {bytes(data).hex()}")

    def log_error(self, direction: str, wvalue: Optional[int], exc: Exception) -> None:
        if wvalue is None:
            self._write(f"! ERR {direction}  {exc}")
        else:
            self._write(f"! ERR {direction} wv=0x{wvalue:04x}  {exc}")

    def annotate(self, text: str) -> None:
        """Write a free-text marker line (e.g. key pressed, jog axis/dist)."""
        for line in text.splitlines():
            self._write(f"# {line}")

    def wrap_link(self, link: MdxLink) -> "_TracingLink":
        """Return an `MdxLink` that logs every transfer call to this tracer.

        Use:
            tracer = open_trace()
            link = tracer.wrap_link(MdxUSB.discover())
            machine = MDX40A(link)

        The wrapper supports the context-manager protocol; on `__exit__` it
        calls `release()` on the inner link.
        """
        return _TracingLink(link, self)

    def close(self) -> None:
        with self._lock:
            if not self._fh.closed:
                elapsed = time.monotonic() - self._start
                self._fh.write(f"# Session end  elapsed={elapsed:.3f}s\n")
                self._fh.close()

    def __enter__(self) -> "Tracer":
        return self

    def __exit__(
        self,
        type_: type[BaseException] | None,
        value: BaseException | None,
        traceback: TracebackType | None,
    ) -> bool | None:
        self.close()
        print(f"Log saved to {self._path}")
        return None

    # ── Internal ──────────────────────────────────────────────────────────────

    def _write(self, text: str) -> None:
        ts = (
            datetime.now().strftime("%H:%M:%S.")
            + f"{datetime.now().microsecond // 1000:03d}"
        )
        with self._lock:
            self._fh.write(f"{ts} {text}\n")


class _TracingLink:
    """`MdxLink` wrapper produced by `Tracer.wrap_link`. Delegates the three
    primitive transfers to the inner link and logs each one. Errors raised by
    the inner link are recorded as `! ERR …` lines and re-raised."""

    def __init__(self, inner: MdxLink, tracer: Tracer):
        self._inner = inner
        self._tracer = tracer

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def release(self) -> None:
        self._inner.release()

    def __enter__(self) -> "_TracingLink":
        return self

    def __exit__(self, *_: Any) -> None:
        self.release()

    # ── Transfer primitives ──────────────────────────────────────────────────

    def vend_set(self, wValue: int, data: bytes = b"") -> None:
        try:
            self._inner.vend_set(wValue, data)
        except Exception as exc:
            self._tracer.log_error("SET", wValue, exc)
            raise
        self._tracer.log_set(wValue, data)

    def vend_get(self, wValue: int, length: int) -> bytes:
        try:
            bs = self._inner.vend_get(wValue, length)
        except Exception as exc:
            self._tracer.log_error("GET", wValue, exc)
            raise
        self._tracer.log_get(wValue, bs)
        return bs

    def bulk_write(self, data: bytes) -> int:
        try:
            n = self._inner.bulk_write(data)
        except Exception as exc:
            self._tracer.log_error("BULK", None, exc)
            raise
        self._tracer.log_bulk(data)
        return n


def open_trace() -> Tracer:
    """Create ~/roland/<timestamp>.txt and return a Tracer for it."""
    directory = Path.home() / "roland"
    directory.mkdir(parents=True, exist_ok=True)
    stamp = (
        datetime.now().strftime("%Y%m%d-%H%M%S.")
        + f"{datetime.now().microsecond // 1000:03d}"
    )
    path = directory / f"{stamp}.txt"
    return Tracer(path)


