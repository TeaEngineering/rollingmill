"""Tests for rollingmill.sleep_inhibit and its integration with CutJob.

The unit tests monkeypatch `subprocess.Popen` so no real `caffeinate`
process is ever spawned — the suite runs identically on Linux CI.
"""
from __future__ import annotations

import os

import pytest

from rollingmill import sleep_inhibit
from rollingmill.cutjob import CutJob
from rollingmill.machine import MDX40A
from rollingmill.usb import MdxMockUSB


class _FakePopen:
    """Stand-in for subprocess.Popen that records terminate() calls."""

    def __init__(self, argv, **_kw):
        self.argv = argv
        self.terminated = False

    def terminate(self) -> None:
        self.terminated = True


@pytest.fixture
def fake_darwin(monkeypatch):
    """Pretend we're on darwin and capture every Popen call. Yields the list."""
    spawned: list[_FakePopen] = []

    def fake_popen(argv, **kw):
        proc = _FakePopen(argv, **kw)
        spawned.append(proc)
        return proc

    monkeypatch.setattr(sleep_inhibit, "sys", _ModWithPlatform("darwin"))
    monkeypatch.setattr(sleep_inhibit.subprocess, "Popen", fake_popen)
    # Reset module state so each test starts clean.
    monkeypatch.setattr(sleep_inhibit, "_count", 0)
    monkeypatch.setattr(sleep_inhibit, "_proc", None)
    yield spawned
    # Drain any outstanding count so subsequent tests aren't polluted.
    while sleep_inhibit.is_active():
        sleep_inhibit.release()


class _ModWithPlatform:
    """Minimal stand-in for the `sys` module exposing only `.platform`."""
    def __init__(self, platform: str) -> None:
        self.platform = platform


def test_refcount_pairs_correctly(fake_darwin):
    sleep_inhibit.acquire()
    sleep_inhibit.acquire()
    assert sleep_inhibit.is_active()
    sleep_inhibit.release()
    assert sleep_inhibit.is_active()
    sleep_inhibit.release()
    assert not sleep_inhibit.is_active()


def test_spurious_release_is_noop(fake_darwin):
    sleep_inhibit.release()
    sleep_inhibit.release()
    assert not sleep_inhibit.is_active()


def test_spawn_happens_once_at_count_one(fake_darwin):
    sleep_inhibit.acquire()
    sleep_inhibit.acquire()
    sleep_inhibit.acquire()
    assert len(fake_darwin) == 1
    assert fake_darwin[0].argv == ["caffeinate", "-i", "-w", str(os.getpid())]


def test_terminate_happens_once_at_count_zero(fake_darwin):
    sleep_inhibit.acquire()
    sleep_inhibit.acquire()
    sleep_inhibit.release()
    assert not fake_darwin[0].terminated
    sleep_inhibit.release()
    assert fake_darwin[0].terminated


def test_non_darwin_no_op(monkeypatch):
    monkeypatch.setattr(sleep_inhibit, "sys", _ModWithPlatform("linux"))
    spawned: list = []
    monkeypatch.setattr(sleep_inhibit.subprocess, "Popen",
                        lambda *a, **kw: spawned.append(a) or _FakePopen(a))
    monkeypatch.setattr(sleep_inhibit, "_count", 0)
    monkeypatch.setattr(sleep_inhibit, "_proc", None)

    sleep_inhibit.acquire()
    sleep_inhibit.acquire()
    sleep_inhibit.release()
    sleep_inhibit.release()
    assert spawned == []


def test_missing_caffeinate_binary_does_not_raise(monkeypatch):
    monkeypatch.setattr(sleep_inhibit, "sys", _ModWithPlatform("darwin"))

    def boom(*a, **kw):
        raise FileNotFoundError("no caffeinate")

    monkeypatch.setattr(sleep_inhibit.subprocess, "Popen", boom)
    monkeypatch.setattr(sleep_inhibit, "_count", 0)
    monkeypatch.setattr(sleep_inhibit, "_proc", None)

    sleep_inhibit.acquire()    # should not raise
    assert sleep_inhibit.is_active()
    sleep_inhibit.release()
    assert not sleep_inhibit.is_active()


# ── CutJob integration ───────────────────────────────────────────────────────

class _AckingMockUSB(MdxMockUSB):
    """Mock that advances the NC bytes-processed counter on each bulk_write."""

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


@pytest.fixture
def counting_inhibitor(monkeypatch):
    """Replace acquire/release on the sleep_inhibit module with pure counters.

    Patches both the module itself and the binding imported into cutjob, so it
    doesn't matter which alias the production code holds.
    """
    counts = {"acquire": 0, "release": 0}

    def acq(reason="rollingmill cut job"):
        counts["acquire"] += 1

    def rel():
        counts["release"] += 1

    from rollingmill import cutjob
    monkeypatch.setattr(sleep_inhibit, "acquire", acq)
    monkeypatch.setattr(sleep_inhibit, "release", rel)
    monkeypatch.setattr(cutjob.sleep_inhibit, "acquire", acq)
    monkeypatch.setattr(cutjob.sleep_inhibit, "release", rel)
    return counts


def _new_job(payload: bytes = b'G0X0\rG0Y0\rG0Z0\r') -> CutJob:
    return CutJob.from_bytes(MDX40A(_AckingMockUSB()), payload, 'test.nc')


def test_cutjob_run_to_done_pairs_acquire_release(counting_inhibitor):
    job = _new_job()
    job.run()
    for _ in range(10):
        job.service()
    assert job.state == CutJob.DONE
    assert counting_inhibitor == {"acquire": 1, "release": 1}


def test_cutjob_abort_releases(counting_inhibitor):
    job = _new_job()
    job.step()
    assert counting_inhibitor["acquire"] == 1 and counting_inhibitor["release"] == 0
    job.abort()
    assert counting_inhibitor == {"acquire": 1, "release": 1}


def test_cutjob_restart_then_rerun_pairs_twice(counting_inhibitor):
    job = _new_job()
    job.run()
    for _ in range(10):
        job.service()
    assert job.state == CutJob.DONE
    assert counting_inhibitor == {"acquire": 1, "release": 1}

    job.restart()
    job.run()
    for _ in range(10):
        job.service()
    assert job.state == CutJob.DONE
    assert counting_inhibitor == {"acquire": 2, "release": 2}


def test_cutjob_idle_does_not_acquire(counting_inhibitor):
    _new_job()   # constructed but never started — bracket never opens
    assert counting_inhibitor == {"acquire": 0, "release": 0}
