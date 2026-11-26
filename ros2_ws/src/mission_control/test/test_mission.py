"""Behavioural checks: complete collection, blocked layouts and failure latching."""

import copy
import math
from pathlib import Path

import pytest

from mission_control.core import Mission
from mission_control.route import Action, compile_route, load_config, pose, validate_config, validate_geometry
from mission_control.validate import rehearse


CONFIG = Path(__file__).resolve().parents[1] / 'config/fixed_route_sim.yaml'


@pytest.fixture
def cfg():
    return load_config(CONFIG)


def test_closed_loop_collects_each_rack_and_exits(cfg):
    mission, elapsed = rehearse(cfg)
    assert mission.state == 'DONE', mission.reason
    assert mission.placed == list('ABCD')
    assert elapsed < cfg['control']['mission_timeout_s']
    assert mission.carrying is None
    assert mission.command == (0.,0.)
    entered = [e for e in mission.events if e['event'] == 'action_enter']
    assert [e['rack'] for e in entered if e['kind'] == 'photo'] == ['START', 'A', 'B', 'C', 'D', 'END']
    assert mission.actions[0].target[2] == pytest.approx(math.atan2(0.30, -0.40))
    assert mission.actions[-1].target[2] == pytest.approx(3 * math.pi / 4)
    for key in cfg['order']:
        states = [e['state'] for e in entered if e['rack'] == key]
        for required in ('TO_PHOTO','DWELL','PHOTO','ENTER','LIFT','CLEAR_RACK','TO_DROP','LOWER','EXIT'):
            assert required in states
        assert states.index('PHOTO') < states.index('ENTER') < states.index('LIFT')
        assert states.index('LOWER') < states.index('EXIT')


def test_photo_failure_stops_before_enter_and_lift(cfg):
    mission = Mission(cfg)
    p = pose(cfg['start_pose'])
    mission.start(0.,p)
    # Drive the feedback controller, but fail the actual camera effect.
    now, velocity = 0.,(0.,0.)
    while mission.state == 'RUNNING' and mission.action.kind != 'photo':
        v,w = mission.tick(now,p,velocity)
        dt = 0.02
        p = (p[0]+v*math.cos(p[2]+w*dt/2)*dt,
             p[1]+v*math.sin(p[2]+w*dt/2)*dt, p[2]+w*dt)
        velocity, now = (v,w), now+dt
    a = mission.action
    mission.acknowledge(a.id,False,'camera disconnected')
    assert mission.state == 'STOPPED'
    assert mission.tick(now+1,p) == (0.,0.)
    assert mission.carrying is None
    assert not mission.start(now+2,p)[0]
    assert not any(e.get('state') == 'ENTER' for e in mission.events)


def test_photo_timeout_override_is_independent_of_lifter_and_waits(cfg):
    cfg['control'].update(photo_first_frame_timeout_s=10., photo_timeout_s=15.)
    actions = compile_route(validate_config(cfg))
    assert {a.timeout for a in actions if a.kind == 'photo'} == {15.}
    assert {a.timeout for a in actions if a.kind in ('lift', 'lower')} == {cfg['control']['effect_timeout_s']}
    assert all(a.timeout == a.value + cfg['control']['effect_timeout_s'] for a in actions if a.kind == 'wait')
    cfg['control']['photo_timeout_s'] = 9.
    with pytest.raises(ValueError, match='First-frame timeout'):
        validate_config(cfg)


def test_hardware_scan_timeout_continues_to_next_action(cfg):
    cfg['simulation_only'] = False
    mission = Mission(cfg)
    assert mission.start(0., pose(cfg['start_pose']))[0]
    mission.index = next(i for i, a in enumerate(mission.actions) if a.kind == 'photo')
    mission._enter(1.)
    scan = mission.action
    deadline = mission.entered + scan.timeout

    assert mission.tick(deadline - 0.01, scan.target) == (0., 0.)
    assert mission.action == scan
    assert mission.tick(deadline, scan.target) == (0., 0.)
    assert mission.tick(deadline + cfg['control']['settle_s'] + 0.01, scan.target) == (0., 0.)
    assert mission.state == 'RUNNING' and mission.action != scan
    assert [e['id'] for e in mission.events if e['event'] == 'photo_skipped'] == [scan.id]
    assert not any(e['event'] == 'effect_complete' for e in mission.events)


@pytest.mark.parametrize('fault', ['odom missing/stale','scan missing/stale','Gazebo cargo adapter not ready/healthy'])
def test_feedback_fault_is_immediate_and_latched(cfg,fault):
    m = Mission(cfg)
    p = pose(cfg['start_pose'])
    m.start(0,p)
    assert m.tick(0.1,p,fault=fault) == (0.,0.)
    assert m.state == 'STOPPED' and m.reason == fault
    assert m.tick(0.2,p) == (0.,0.)


def test_no_progress_and_deadline_are_independent_of_waits(cfg):
    m = Mission(cfg)
    p = pose(cfg['start_pose'])
    m.start(0,p)
    for i in range(1,40):
        m.tick(i/10,p)
    assert m.state == 'STOPPED' and 'no progress' in m.reason
    m.reset()
    m.start(0,p)
    m.index = next(i for i,a in enumerate(m.actions) if a.kind == 'wait')
    m._enter(cfg['control']['mission_timeout_s']-0.1)
    assert m.tick(cfg['control']['mission_timeout_s'],m.action.start) == (0.,0.)
    assert m.reason == 'mission deadline exceeded'


def test_localization_pause_keeps_action_and_restarts_velocity_ramp(cfg):
    mission = Mission(cfg)
    start = pose(cfg['start_pose'])
    assert mission.start(0., start)[0]
    action = mission.action
    assert mission.tick(0.1, start)[1] != 0.

    assert mission.tick(0.2, (0., 0., 0.), paused=True) == (0., 0.)
    assert mission.tick(5., (0., 0., 0.), paused=True) == (0., 0.)
    assert mission.state == 'RUNNING' and mission.action == action

    mission.resume_after_pause(5., 0.2)
    assert mission.tick(5., start) == (0., 0.)
    assert mission.tick(5.02, start)[1] != 0.
    assert mission.entered == pytest.approx(4.8)


def test_cancel_while_carrying_preserves_support_and_ignores_late_effect(cfg):
    m = Mission(cfg)
    m.start(0,pose(cfg['start_pose']))
    m.carrying = 'A'
    action_id = m.action.id
    index = m.index
    m.stop('operator cancel')
    m.acknowledge(action_id,True,'late service completion')
    assert m.carrying == 'A' and m.index == index and not m.effect_done
    assert m.command == (0.,0.)


def test_turn_crosses_pi_boundary_in_short_direction(cfg):
    m = Mission(cfg)
    start, target = (1.,1.,math.pi-0.1),(1.,1.,-math.pi+0.1)
    m.actions = [Action('wrap','A','TO_PHOTO','turn',start,target,0.2,1.,10.)]
    m.state, m.started, m.last_tick = 'RUNNING',0.,0.
    m._enter(0.)
    v,w = m.tick(0.1,start)
    assert v == 0 and w > 0  # +0.2 rad, not a nearly full clockwise revolution.
    m.tick(0.2,target)
    m.tick(0.4,target)
    assert m.state == 'DONE' and m.command == (0.,0.)


def test_placed_racks_and_marker_are_checked(cfg):
    # A marker inside the pickup lane obstructs the approach.
    bad = copy.deepcopy(cfg)
    bad['scene']['markers']['E'].update(x=0.55, y=1.75)
    with pytest.raises(ValueError,match='marker E'):
        validate_geometry(bad)
    bad = copy.deepcopy(cfg)
    bad['racks']['C']['drop_pose'] = dict(bad['racks']['A']['drop_pose'])
    with pytest.raises(ValueError,match='rack [AB]|overlap'):
        validate_geometry(bad)


def test_default_rack_approach_and_departure_directions(cfg):
    actions = compile_route(cfg)
    for key in cfg['order']:
        r = cfg['racks'][key]
        north_approach = key == 'D'
        assert (r['photo_pose']['y'] > r['pose']['y']) == north_approach
        assert r['photo_pose']['yaw'] == pytest.approx(-math.pi/2 if north_approach else math.pi/2)
        enter = next(a for a in actions if a.rack == key and a.state == 'ENTER')
        clear = next(a for a in actions if a.rack == key and a.state == 'CLEAR_RACK')
        exit_action = next(a for a in actions if a.rack == key and a.state == 'EXIT')
        assert (enter.target[1] < enter.start[1]) == north_approach
        assert (clear.value < 0) == north_approach
        assert clear.target[1] > clear.start[1]
        assert exit_action.value < 0  # Unloading still needs to release the forks.


def test_legacy_config_still_defaults_to_backward_clear(cfg):
    for r in cfg['racks'].values():
        r.pop('clear_direction', None)
    assert all(a.value < 0 for a in compile_route(cfg) if a.state == 'CLEAR_RACK')
    cfg['racks']['A']['clear_direction'] = 'sideways'
    with pytest.raises(ValueError,match='clear_direction'):
        validate_config(cfg)


@pytest.mark.parametrize('edit', ['null','infinity','missing_point','duplicate_order','missed_pickup','outside_drop'])
def test_invalid_configuration_is_rejected(cfg,edit):
    if edit == 'null':
        cfg['racks']['A']['enter_distance_m'] = None
    elif edit == 'infinity':
        cfg['control']['segment_timeout_s'] = math.inf
    elif edit == 'missing_point':
        cfg['racks']['A']['to_photo'] = ['unknown']
    elif edit == 'duplicate_order':
        cfg['order'] = ['A','A','C','D']
    elif edit == 'missed_pickup':
        cfg['racks']['A']['enter_distance_m'] = 0.10
    else:
        cfg['racks']['A']['drop_pose']['x'] = 0.1
    with pytest.raises(ValueError):
        validate_config(cfg)
