"""Regression coverage for the published geometry and the clearance-constrained mission."""

from copy import deepcopy
import math
from pathlib import Path

import pytest

from mission_control.clearance import SceneClearance, curve_violation, drive_violation
from mission_control.core import Mission
from mission_control.curve import samples
from mission_control.field_map import rasterize
from mission_control.motion import curve_request, motion_request
from mission_control.offline_plan import plan_route
from mission_control.route import action_samples, collision, compose, load_config, pose, rack_polygons, validate_geometry
from mission_control.validate import rehearse


@pytest.fixture(scope='module')
def source():
    return load_config(Path(__file__).resolve().parents[1] / 'config/fixed_route_sim.yaml')


@pytest.fixture(scope='module')
def planned(source):
    return plan_route(source)


def test_search_retains_object_locations_and_completes(source, planned):
    assert planned['order'] == source['order']
    for key, rack in planned['racks'].items():
        assert rack['pose'] == source['racks'][key]['pose']
        assert rack['drop_pose'] == source['racks'][key]['drop_pose']
    rack_d = planned['racks']['D']
    assert rack_d['photo_pose']['y'] > rack_d['pose']['y']
    assert rack_d['photo_pose']['yaw'] == pytest.approx(-math.pi / 2)
    for action in validate_geometry(planned):
        if action.rack == 'D' and action.state == 'TO_PHOTO':
            assert all(point[1] > rack_d['pose']['y'] for point in action_samples(action))
    mission, elapsed = rehearse(planned)
    assert mission.state == 'DONE', mission.reason
    assert elapsed < planned['control']['mission_timeout_s']
    assert mission.placed == source['order']


def test_current_layout_has_one_continuous_curve_for_each_transit(planned):
    actions = validate_geometry(planned)
    curves = [action for action in actions if action.kind == 'curve']
    assert {(action.rack, action.state) for action in curves} == {
        (rack, stage) for rack in planned['order'] for stage in ('TO_PHOTO', 'TO_DROP')} | {('END', 'TO_END')}
    assert len(curves) == 9
    assert sum(action.kind == 'turn' for action in actions) < 12
    for action in curves:
        samples = list(action_samples(action))
        assert math.dist(samples[-1][:2], action.target[:2]) < 1e-8
        chord = math.dist(action.start[:2], action.target[:2])
        assert action.value > chord and any(k for _, k in action.segments)
    assert all(action.kind == 'drive' for action in actions if action.state in ('ENTER', 'CLEAR_RACK', 'EXIT'))


def test_encoder_route_executes_offset_docking_and_recovers_terminal_overshoot():
    root = Path(__file__).resolve().parents[4]
    cfg = load_config(root / 'sim/routes/encoder_a_offset.yaml')
    actions = validate_geometry(cfg)
    rack_actions = [action for action in actions if action.rack == 'A']
    photo = next(action for action in rack_actions if action.kind == 'photo')
    enter = next(action for action in rack_actions if action.state == 'ENTER')
    clear = next(action for action in rack_actions if action.state == 'CLEAR_RACK')
    assert photo.target[0] == pytest.approx(0.500)
    assert enter.target[0] == pytest.approx(photo.target[0])
    mission = Mission(cfg)
    stopped_pose = (0.501217828212078, 1.689376073108336, 1.5645269350923559)
    solid = deepcopy(cfg)
    solid['scene'].pop('racks_passable')
    assert collision(solid, stopped_pose, mission.racks) == 'rack A'
    assert collision(cfg, stopped_pose, mission.racks) is None
    assert SceneClearance(cfg, mission.racks).violation(stopped_pose, 0.10) is None
    assert collision(cfg, (0.30, 2.22, 0.0), mission.racks) == 'marker E'
    assert collision(cfg, (-0.10, 1.0, 0.0), mission.racks) == 'field boundary'
    assert mission.start(0., pose(cfg['start_pose']))[0]
    mission.index = actions.index(enter)
    mission._enter(0.)
    assert mission.tick(0.1, stopped_pose)[0] > 0.
    assert motion_request(clear, clear.start, cfg['control'])[0][0] > 0.
    assert motion_request(enter, compose(enter.target, (0.035, 0, 0)), cfg['control'])[3] == 'overshot target'
    curve = next(action for action in actions if action.kind == 'curve')
    path = tuple(zip(*samples(curve.start, curve.segments)))
    pose_past_end = compose(curve.target, (0.01227, 0.00462, 0.0494))
    (speed, turn), done, _, fault = curve_request(curve, path, curve.value, pose_past_end, cfg['control'])
    assert speed == 0. and turn < 0. and not done and fault is None
    assert curve_request(curve, path, curve.value, compose(curve.target, (0.19, 0, 0)), cfg['control'])[1]
    assert curve_request(curve, path, curve.value, compose(curve.target, (0.21, 0, 0)), cfg['control'])[3] == 'overshot curve target'
    mission, _ = rehearse(cfg)
    assert mission.state == 'DONE', mission.reason
    assert mission.placed == cfg['order']


def test_curve_sweep_checks_cargo_between_endpoints(source):
    cfg = deepcopy(source)
    cfg['scene']['markers'] = {'S': {'x': 0.82, 'y': 0.78, 'radius': 0.025}}
    start = (0.55, 0.65, 0.)
    segments = ((0.4 * math.pi / 2, 1 / 0.4),)
    scene = SceneClearance(cfg, {}, 'A')
    assert scene.violation(start, 0.005) is None
    assert scene.violation((0.95, 1.05, math.pi / 2), 0.005) is None
    assert curve_violation(scene, start, segments, 0.005) == 'marker S'


def test_old_route_fails_requested_clearance(source):
    with pytest.raises(ValueError, match=r'A_TO_PHOTO.*rack B'):
        validate_geometry({**source, 'planning': {'clearance_m': 0.10}})


def test_docking_exception_does_not_exempt_columns_or_other_racks(planned):
    racks = {key: pose(rack['pose']) for key, rack in planned['racks'].items()}
    scene = SceneClearance(planned, racks)
    pickup = (racks['A'][0], racks['A'][1], math.pi / 2)
    assert scene.violation(pickup, 0.10) == 'rack A'
    assert scene.violation(pickup, 0.10, docking='A') is None
    assert scene.violation((0.41, 0.80, math.pi / 2), 0.10, docking='D') == 'marker S'
    assert scene.violation(pickup, 0.10, docking='D') == 'rack A'


def test_swept_cargo_projects_the_forward_drive_across_the_rack(source):
    cfg = deepcopy(source)
    cfg['scene']['markers'] = {'S': {'x': 1.18, 'y': 1.36, 'radius': 0.05}}
    start = (1., 0.75, math.pi / 2)
    end = (1., 1.25, math.pi / 2)
    assert drive_violation(SceneClearance(cfg, {}), start, end, 0.003) is None
    assert drive_violation(SceneClearance(cfg, {}, 'A'), start, end, 0.003) == 'marker S'


def test_runtime_stops_if_odometry_enters_the_clearance(planned):
    mission = Mission(planned)
    assert mission.start(0., pose(planned['start_pose']))[0]
    mission.index = next(i for i, a in enumerate(mission.actions) if a.state == 'CLEAR_RACK')
    mission.carrying = 'A'
    mission._enter(0.1)
    assert mission.tick(0.2, (0.55, 2.055, math.pi / 2)) == (0., 0.)
    assert mission.state == 'STOPPED'
    assert 'marker E inside 0.100 m clearance' in mission.reason


def test_raster_matches_walls_and_keeps_the_rack_channel_open(source, tmp_path):
    a = pose(source['racks']['A']['pose'])
    static = rasterize(source['scene'])
    with_rack = rasterize(source['scene'], {'A': a})

    def occupied(grid, x, y):
        i = math.floor((x - grid.origin[0]) / grid.resolution)
        j = math.floor((y - grid.origin[1]) / grid.resolution)
        return grid.cells[j * grid.width + i] == 100

    assert occupied(static, -0.025, 1.)
    assert occupied(static, 0.6, 0.6)
    assert not occupied(with_rack, *a[:2])
    for polygon in rack_polygons(source, a):
        x, y = (sum(coords) / 4 for coords in zip(*polygon))
        assert not occupied(static, x, y)
        assert occupied(with_rack, x, y)
    static.save(tmp_path / 'field.yaml')
    assert (tmp_path / 'field.pgm').read_bytes().startswith(b'P5\n210 312\n255\n')
