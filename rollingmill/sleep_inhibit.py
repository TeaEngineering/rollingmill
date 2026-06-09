"""Hold the OS awake while a long-running operation is in progress.

On macOS, `acquire()` spawns `caffeinate -i -w <our_pid>` and `release()`
terminates it. The `-w` form means caffeinate auto-exits if our process
dies — the OS sleep assertion is released cleanly even on a hard crash.

On other platforms both calls are no-ops. Linux equivalents could shell out
to `systemd-inhibit`; Windows could call `SetThreadExecutionState` via
ctypes — neither is implemented yet.

Refcounted so multiple concurrent users (e.g. a future probe sequence
overlapping a cut job) compose correctly. Module-level singleton — there is
exactly one OS-level assertion per process.
"""

import os
import subprocess
import sys
import threading
from typing import Optional

_lock = threading.Lock()
_count = 0
_proc: Optional[subprocess.Popen] = None


def acquire(reason: str = "rollingmill cut job") -> None:
    """Hold the system awake. Idempotent under refcount; pair with release()."""
    global _count, _proc
    with _lock:
        _count += 1
        if _count == 1 and sys.platform == "darwin":
            try:
                _proc = subprocess.Popen(
                    ["caffeinate", "-i", "-w", str(os.getpid())],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except FileNotFoundError:
                _proc = None


def release() -> None:
    """Drop one acquire. When the count reaches zero, release the assertion."""
    global _count, _proc
    with _lock:
        if _count == 0:
            return
        _count -= 1
        if _count == 0 and _proc is not None:
            _proc.terminate()
            _proc = None


def is_active() -> bool:
    """True while at least one acquire is outstanding."""
    with _lock:
        return _count > 0
