"""Ensure editing the route YAML changes the actual Gazebo world geometry."""

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

from mission_control.route import load_config
from sim_control.mission_world import render_world


ROOT = Path(__file__).resolve().parents[4]


def test_yaml_scene_and_resource_paths_are_applied_without_editing_source(tmp_path):
    source = ROOT / 'sim/worlds/dase7503_field.sdf'
    cfg = load_config(ROOT/'ros2_ws/src/mission_control/config/fixed_route_sim.yaml')
    before = hashlib.sha256(source.read_bytes()).digest()
    cfg['racks']['A']['pose'].update(x=0.7,y=1.8,yaw=0.3)
    cfg['scene']['rack']['gap'] = 0.40
    cfg['scene']['rack']['depth'] = 0.20
    cfg['scene']['markers']['S'].update(x=0.8,radius=0.08)
    target = tmp_path/'world.sdf'
    render_world(source,target,cfg)
    root = ET.parse(target).getroot()
    rack = root.find("world/model[@name='rack_a']")
    assert list(map(float,rack.find('pose').text.split())) == [0.7,1.8,0,0,0,0.3]
    assert float(rack.find("link[@name='east_wall']/pose").text.split()[0]) == pytest.approx(0.2125)
    assert list(map(float,rack.find('.//box/size').text.split())) == [0.025,0.2,0.23]
    marker = root.find("world/model[@name='marker_s']")
    assert float(marker.find('pose').text.split()[0]) == 0.8
    assert float(marker.find('.//cylinder/radius').text) == 0.08
    for tag in ('uri','albedo_map'):
        for node in root.iter(tag):
            assert Path(node.text).is_absolute() and Path(node.text).exists()
    assert hashlib.sha256(source.read_bytes()).digest() == before
