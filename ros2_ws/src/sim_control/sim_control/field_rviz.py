"""Add field map and route displays to the shared simulation RViz view."""

from pathlib import Path

import yaml


def write_field_rviz(template, target, field, *, mission):
    cfg = yaml.safe_load(Path(template).read_text(encoding='utf-8'))
    cfg['Panels'] = [panel for panel in cfg['Panels'] if panel['Name'] != 'Views']
    cfg['Window Geometry'] = {'Width': 1120, 'Height': 820}
    manager = cfg['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = 'map'
    view = manager['Views']['Current']
    view.update(Scale=150., X=(field['x_min'] + field['x_max']) / 2,
                Y=(field['y_min'] + field['y_max']) / 2)
    displays = manager['Displays']
    for display in displays:
        if display['Name'] in ('TF', 'Front camera'):
            display['Enabled'] = False

    def topic(name):
        return {'Value': name, 'Depth': 1, 'Durability Policy': 'Transient Local',
                'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable'}

    def map_display(name, name_topic, enabled):
        displays.append({'Class': 'rviz_default_plugins/Map', 'Name': name,
                         'Enabled': enabled, 'Topic': topic(name_topic),
                         'Draw Behind': True, 'Alpha': 1.0, 'Use Timestamp': False})

    map_display('Saved field map' if mission else 'LiDAR SLAM map', '/map', True)
    if mission:
        map_display('Field and stationary racks', '/mission/scene_map', False)
        displays.append({'Class': 'rviz_default_plugins/PoseWithCovariance',
                         'Name': 'AMCL pose', 'Enabled': True, 'Topic': topic('/amcl_pose'),
                         'Shape': 'Arrow', 'Shaft Length': 0.15, 'Head Length': 0.07})
        manager['Tools'].append({'Class': 'rviz_default_plugins/SetInitialPose', 'Topic': '/initialpose'})
        for name, path_topic, color, width in (
            ('Planned mission route', '/mission/planned_path', '94; 91; 183', 0.020),
            ('Current route segment', '/mission/active_path', '0; 218; 185', 0.038),
        ):
            displays.append({'Class': 'rviz_default_plugins/Path', 'Name': name, 'Enabled': True,
                             'Topic': topic(path_topic), 'Color': color,
                             'Line Style': 'Billboards', 'Line Width': width,
                             'Pose Style': 'Axes' if path_topic.endswith('active_path') else 'None',
                             'Length': 0.08, 'Radius': 0.008})
    target = Path(target)
    target.write_text(yaml.safe_dump(cfg, sort_keys=False), encoding='utf-8')
    return str(target)
