"""CutJob state-machine tests.

Drives `CutJob` over an `MdxMockUSB` subclass that fakes the NC bytes-processed
counter (GET 0x0200): `bulk_write` advances the counter by `len(data)` so the
next poll matches the expected value and the job acks immediately, mirroring
the firmware's per-block ack on real hardware.
"""

from mdx40a.cutjob import CutJob
from mdx40a.machine import MDX40A
from mdx40a.usb import MdxMockUSB


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
