"""Nonblocking mission and local feedback controller. No ROS/Gazebo imports."""

import math

from .motion import motion_request, ramp_command
from .route import angle, cargo_offset, collision, compose, pose, validate_geometry


class Mission:
    def __init__(self, cfg):
        self.cfg = cfg
        self.actions = validate_geometry(cfg)
        self.events = []
        self.reset()

    def emit(self, event, **fields):
        self.events.append(dict(event=event, **fields))

    @property
    def action(self):
        return self.actions[self.index] if self.index < len(self.actions) else None

    def reset(self):
        self.state = 'IDLE'
        self.reason = ''
        self.index = 0
        self.carrying = None
        self.racks = {k: pose(r['pose']) for k, r in self.cfg['racks'].items()}
        self.placed = []
        self.started = None
        self.entered = None
        self.command = (0., 0.)
        self.effect_done = False
        self.settled_since = None
        self.last_tick = None
        self.best_error = math.inf
        self.last_progress = None
        self.emit('reset')

    def start(self, now, p):
        if self.state != 'IDLE':
            return False, 'Reset the scene before another run'
        start = pose(self.cfg['start_pose'])
        if math.hypot(p[0]-start[0], p[1]-start[1]) > 0.025 or abs(angle(p[2]-start[2])) > 0.05:
            return False, 'Robot is not at configured start pose; reset scene'
        self.started = now
        self.last_tick = now
        self.state = 'RUNNING'
        self.emit('start', version=self.cfg['version'], order=self.cfg['order'])
        self._enter(now)
        return True, 'Started'

    def _enter(self, now):
        self.entered = now
        self.effect_done = False
        self.settled_since = None
        self.best_error = math.inf
        self.last_progress = now
        a = self.action
        self.emit('action_enter', id=a.id, rack=a.rack, state=a.state, kind=a.kind,
                  target=list(a.target), value=a.value)

    def stop(self, reason):
        self.command = (0., 0.)
        if self.state == 'RUNNING':
            self.state = 'STOPPED'
            self.reason = reason
            self.emit('stopped', reason=reason, id=self.action.id, carrying=self.carrying)

    def acknowledge(self, action_id, success, detail=''):
        if self.state != 'RUNNING' or self.action.id != action_id:
            return  # Ignore late effects after cancellation/reset.
        if not success:
            self.stop(f'{action_id}: {detail}')
        else:
            self.effect_done = True
            self.emit('effect_complete', id=action_id, detail=detail)

    def tick(self, now, p, velocity=(0., 0.), fault=None):
        if self.state != 'RUNNING':
            return (0., 0.)
        if fault:
            self.stop(fault)
            return self.command
        c = self.cfg['control']
        a = self.action
        if now - self.started >= c['mission_timeout_s']:
            self.stop('mission deadline exceeded')
            return self.command
        if now - self.entered >= a.timeout:
            self.stop(f'{a.id}: action timeout')
            return self.command
        hit = collision(self.cfg, p, self.racks, self.carrying)
        if hit:
            self.stop(f'{a.id}: predicted footprint intersects {hit}')
            return self.command

        v, w = 0., 0.
        done = False
        error = 0.
        if a.kind in ('turn', 'drive'):
            (v, w), done, error, motion_fault = motion_request(a, p, c)
            if motion_fault:
                self.stop(f'{a.id}: {motion_fault}')
                return self.command
        elif a.kind == 'wait':
            done = now - self.entered >= a.value
        else:
            done = self.effect_done

        if a.kind in ('drive', 'turn') and not done:
            if error < self.best_error - 0.001:
                self.best_error = error
                self.last_progress = now
            elif now - self.last_progress > c['no_progress_s']:
                self.stop(f'{a.id}: no progress')
                return self.command

        dt = min(0.1, max(0., now - self.last_tick))
        self.last_tick = now
        if not done and a.kind in ('drive', 'turn'):
            v, w = ramp_command(self.command, (v, w), dt, c)
        self.command = (v, w)
        if done:
            self.command = (0., 0.)
            stopped = abs(velocity[0]) <= c['stopped_linear_mps'] and abs(velocity[1]) <= c['stopped_angular_rps']
            if stopped:
                if self.settled_since is None:
                    self.settled_since = now
                if now - self.settled_since >= c['settle_s']:
                    self._advance(now, p)
            else:
                self.settled_since = None
        return self.command

    def _advance(self, now, p):
        a = self.action
        if a.kind == 'lift':
            self.carrying = a.rack
        elif a.kind == 'lower':
            self.racks[a.rack] = compose(p, cargo_offset(self.cfg['racks'][a.rack]))
            self.placed.append(a.rack)
            self.carrying = None
        self.emit('action_exit', id=a.id, rack=a.rack, state=a.state, pose=list(p))
        self.index += 1
        if self.index == len(self.actions):
            self.state = 'DONE'
            self.emit('done', elapsed_s=now-self.started, placed=list(self.placed))
        else:
            self._enter(now)
