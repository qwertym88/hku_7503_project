from glob import glob
import os
from pathlib import Path
from setuptools import setup

package_name = 'robot_navigation'
package_root = Path(__file__).resolve().parent
standard_map = package_root.parents[2] / 'sim' / 'maps'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
        (f'share/{package_name}/maps', [os.path.relpath(standard_map / name, package_root)
                                       for name in ('field_geometry.yaml', 'field_geometry.pgm')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DASE7503 project team',
    maintainer_email='team@dase7503.invalid',
    description='M4 navigation: lidar -> SLAM -> localization -> Nav2',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'mock_odom = robot_navigation.mock_odom:main',
            'waypoint_driver = robot_navigation.waypoint_driver:main',
        ],
    },
)
