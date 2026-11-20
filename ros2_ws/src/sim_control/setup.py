from glob import glob
import os
from pathlib import Path
from setuptools import setup

package_name = 'sim_control'
project_root = Path(__file__).resolve().parents[3]
sim_root = project_root / 'sim'
sim_assets = [
    ('worlds', [sim_root / 'worlds' / 'dase7503_field.sdf']),
    ('models/ideal_robot', [sim_root / 'models' / 'ideal_robot' / 'model.sdf']),
    ('meshes', [sim_root / 'meshes' / 'qr_panel.dae']),
    ('materials/textures', sorted((sim_root / 'materials' / 'textures').glob('*.png'))),
]
sim_assets = [(directory, [os.path.relpath(path, Path(__file__).parent)
                           for path in files])
              for directory, files in sim_assets]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        *[(f'share/{package_name}/{directory}', files)
          for directory, files in sim_assets],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DASE7503 project team',
    maintainer_email='team@dase7503.invalid',
    description='Ideal kinematic Gazebo base and simple /cmd_vel control panel',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'ideal_base = sim_control.ideal_base:main',
            'control_panel = sim_control.control_panel:main',
        ],
    },
)
