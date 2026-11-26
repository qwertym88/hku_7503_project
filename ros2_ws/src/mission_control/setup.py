from glob import glob
from setuptools import setup

setup(
    name='mission_control', version='0.1.0', packages=['mission_control'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mission_control']),
        ('share/mission_control', ['package.xml']),
        ('share/mission_control/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='DASE7503 project team', maintainer_email='team@dase7503.invalid',
    description='Fixed-route mission with Gazebo and firmware cargo integration',
    license='Proprietary',
    entry_points={'console_scripts': [
        'fixed_route = mission_control.node:main',
        'fixed_route_hardware = mission_control.hardware:main',
        'fixed_route_encoder = mission_control.encoder_hardware:main',
        'chassis_compare = mission_control.chassis_compare:main',
        'chassis_segment = mission_control.chassis_segment:main',
        'hardware_self_check = mission_control.hardware_self_check:main',
        'validate_route = mission_control.validate:main',
    ]},
)
