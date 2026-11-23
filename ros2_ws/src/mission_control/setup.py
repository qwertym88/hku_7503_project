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
    description='Fixed-route mission with simulation-only cargo and camera integration',
    license='Proprietary',
    entry_points={'console_scripts': [
        'fixed_route = mission_control.node:main',
        'validate_route = mission_control.validate:main',
    ]},
)
