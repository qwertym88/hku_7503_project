"""Start the stationary-field SLAM stack and run an automatic Gazebo survey."""

from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    share = Path(get_package_share_directory('sim_control'))
    value = LaunchConfiguration('output').perform(context)
    output = (Path(value).expanduser().resolve() if value else
              Path(tempfile.mkdtemp(prefix='dase7503-field-map-')) / 'field_lidar')
    if output.suffix in ('.yaml', '.pgm'):
        raise ValueError('output must be a filename prefix without .yaml or .pgm')
    output.parent.mkdir(parents=True, exist_ok=True)
    for extension in ('.yaml', '.pgm'):
        target = output.with_name(output.name + extension)
        if target.exists() or target.is_symlink():
            raise FileExistsError(f'Map output already exists: {target}')

    survey = Node(package='sim_control', executable='map_sweep', output='screen',
                  parameters=[{'use_sim_time': True, 'output': str(output),
                               'config': ParameterValue(LaunchConfiguration('config'), value_type=str)}])

    def survey_exited(event, context):
        if context.is_shutdown:
            return []
        if event.returncode:
            # Raise through launch so command-line callers also see the failure.
            raise RuntimeError(f'Field map survey failed at {output} (exit {event.returncode})')
        return [EmitEvent(event=Shutdown(reason=f'Field survey saved {output}.yaml'))]

    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/field_slam.launch.py')),
            launch_arguments=({name: LaunchConfiguration(name)
                              for name in ('config', 'gui', 'rviz', 'ros_domain_id')}
            | {'panel': 'false'}).items()),
        RegisterEventHandler(OnProcessExit(target_action=survey, on_exit=survey_exited)),
        survey,
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('mission_control')) / 'config/fixed_route_sim.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        DeclareLaunchArgument('output', default_value='',
                              description='Output prefix; defaults to a new directory under /tmp'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ros_domain_id', default_value='78'),
        OpaqueFunction(function=start),
    ])
