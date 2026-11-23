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
    # Advancing too far with A passes too close to E before turning toward drop.
    bad = copy.deepcopy(cfg)
    bad['racks']['A']['clear_distance_m'] = 0.35
    with pytest.raises(ValueError,match='marker E'):
        validate_geometry(bad)
    bad = copy.deepcopy(cfg)
    bad['racks']['C']['drop_pose'] = dict(bad['racks']['A']['drop_pose'])
    with pytest.raises(ValueError,match='rack [AB]|overlap'):
        validate_geometry(bad)


def test_default_approaches_from_south_and_leaves_forward(cfg):
    actions = compile_route(cfg)
    for key in cfg['order']:
        r = cfg['racks'][key]
        assert r['photo_pose']['y'] < r['pose']['y']
        assert r['photo_pose']['yaw'] == pytest.approx(math.pi/2)
        enter = next(a for a in actions if a.rack == key and a.state == 'ENTER')
        clear = next(a for a in actions if a.rack == key and a.state == 'CLEAR_RACK')
        exit_action = next(a for a in actions if a.rack == key and a.state == 'EXIT')
        assert enter.target[1] > enter.start[1]
        assert clear.value > 0 and clear.target[1] > clear.start[1]
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
