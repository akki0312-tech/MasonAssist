import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from job_queue import HISTORY_MAX, JobQueue  # noqa: E402


def _start_next(q, now):
    job = q.next_to_start()
    if job is not None:
        q.mark_started(job, now)
    return job


def test_jobs_run_in_fifo_order_one_at_a_time():
    q = JobQueue()
    a = q.add('bricks_to_building', 0.0)
    b = q.add('blocks_to_scaffold', 1.0)
    assert _start_next(q, 2.0) is a
    assert q.next_to_start() is None            # one running already
    q.finish(a, 'complete', None, 0, 10.0)
    assert not q.paused
    assert _start_next(q, 10.0) is b


def test_unknown_mission_rejected():
    with pytest.raises(ValueError):
        JobQueue().add('make_tea', 0.0)


def test_remove_and_clear_only_touch_queued_jobs():
    q = JobQueue()
    a = q.add('bricks_to_building', 0.0)
    b = q.add('blocks_to_scaffold', 0.0)
    c = q.add('bricks_to_building', 0.0)
    _start_next(q, 1.0)                         # a running
    with pytest.raises(ValueError):
        q.remove(a.id)
    q.remove(b.id)
    assert [j.id for j in q.queued] == [c.id]
    q.clear()
    assert q.queued == [] and q.current is a


@pytest.mark.parametrize('status', ['failed', 'aborted'])
def test_failure_or_abort_pauses_queue_until_resumed(status):
    q = JobQueue()
    a = q.add('bricks_to_building', 0.0)
    b = q.add('blocks_to_scaffold', 0.0)
    _start_next(q, 1.0)
    q.finish(a, status, 'something went wrong', 2, 50.0)
    assert q.paused and f'job #{a.id} {status}' in q.pause_reason
    assert q.next_to_start() is None
    q.resume()
    assert _start_next(q, 60.0) is b


def test_operator_pause_holds_next_job():
    q = JobQueue()
    q.add('bricks_to_building', 0.0)
    q.pause()
    assert q.next_to_start() is None
    q.resume()
    assert q.next_to_start() is not None


def test_history_newest_first_and_capped():
    q = JobQueue()
    for i in range(HISTORY_MAX + 5):
        j = q.add('bricks_to_building', float(i))
        _start_next(q, float(i))
        q.finish(j, 'complete', None, 0, float(i) + 1)
    assert len(q.history) == HISTORY_MAX
    assert q.history[0].id == HISTORY_MAX + 5


def test_to_dict_timings():
    q = JobQueue()
    a = q.add('bricks_to_building', 10.0)
    q.add('blocks_to_scaffold', 10.0)
    _start_next(q, 15.0)
    d = q.to_dict(25.0)
    assert d['current']['wait_s'] == 5.0 and d['current']['elapsed_s'] == 10.0
    assert d['queued'][0]['wait_s'] == 15.0 and d['queued'][0]['elapsed_s'] is None
    q.finish(a, 'complete', None, 1, 40.0)
    h = q.to_dict(99.0)['history'][0]
    assert h['status'] == 'complete' and h['elapsed_s'] == 25.0 and h['replans'] == 1
