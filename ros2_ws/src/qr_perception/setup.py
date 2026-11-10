from glob import glob

from setuptools import setup

package_name = 'qr_perception'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DASE7503 project team',
    maintainer_email='team@dase7503.invalid',
    description='M3 perception: picamera2 capture + pure-Python QR decode + thin ROS wrapper publishing /qr',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'qr_perception = qr_perception.qr_node:main',
        ],
    },
)
