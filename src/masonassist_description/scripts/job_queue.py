#!/usr/bin/env python3
"""ROS-free transport job queue + history for MasonAssist.

Jobs are transport missions (transport_mission.MISSIONS) queued by the
operator and run back to back by mission_server.py. Rule: a failed or
operator-aborted job PAUSES the queue - the next job never starts on its own
after something went wrong; the operator reviews and resumes. In-memory only.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from transport_mission import MISSIONS  # noqa: E402

HISTORY_MAX = 50


class Job:

    def __init__(self, job_id, mission, now, spec=None, points=None, label=None):
        self.id = job_id
        self.mission = mission
        self.spec = spec             # custom jobs: legs/material (else MISSIONS[mission])
        self.points = points
        self.label = label or mission
        self.status = 'queued'
        self.queued_at = now
        self.started_at = None
        self.finished_at = None
        self.reason = None
        self.replans = 0

    def to_dict(self, now):
        start = self.started_at
        end = self.finished_at if self.finished_at is not None else now
        return {
            'id': self.id,
            'mission': self.mission,
            'label': self.label,
            'status': self.status,
            'reason': self.reason,
            'replans': self.replans,
            'wait_s': round((start if start is not None else now) - self.queued_at, 1),
            'elapsed_s': None if start is None else round(end - start, 1),
        }


class JobQueue:

    def __init__(self):
        self._next_id = 1
        self.queued = []
        self.current = None
        self.history = []          # newest first
        self.paused = False
        self.pause_reason = None

    def add(self, mission, now, spec=None, points=None, label=None):
        """A MISSIONS name, or 'custom' with a spec (transport_mission.custom_job_spec)."""
        if spec is None and mission not in MISSIONS:
            raise ValueError(f'unknown mission {mission!r}')
        job = Job(self._next_id, mission, now, spec, points, label)
        self._next_id += 1
        self.queued.append(job)
        return job

    def remove(self, job_id):
        for job in self.queued:
            if job.id == job_id:
                self.queued.remove(job)
                return
        raise ValueError(f'job {job_id} is not queued (only queued jobs can be removed)')

    def clear(self):
        self.queued = []

    def pause(self, reason='paused by operator'):
        self.paused = True
        self.pause_reason = reason

    def resume(self):
        self.paused = False
        self.pause_reason = None

    def next_to_start(self):
        if self.paused or self.current is not None or not self.queued:
            return None
        return self.queued[0]

    def mark_started(self, job, now):
        self.queued.remove(job)
        job.status = 'running'
        job.started_at = now
        self.current = job

    def finish(self, job, status, reason, replans, now):
        job.status = status
        job.reason = reason
        job.replans = replans
        job.finished_at = now
        if self.current is job:
            self.current = None
        self.history.insert(0, job)
        del self.history[HISTORY_MAX:]
        if status in ('failed', 'aborted'):
            self.pause(f'job #{job.id} {status}: {reason}')

    def to_dict(self, now):
        return {
            'paused': self.paused,
            'pause_reason': self.pause_reason,
            'current': self.current.to_dict(now) if self.current else None,
            'queued': [j.to_dict(now) for j in self.queued],
            'history': [j.to_dict(now) for j in self.history],
        }
