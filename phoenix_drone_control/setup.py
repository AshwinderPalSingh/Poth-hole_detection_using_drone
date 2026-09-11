from setuptools import setup
import os
from glob import glob

package_name = 'phoenix_drone_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PhoenixDrone Dev',
    maintainer_email='dev@phoenix-drone.local',
    description='PID flight control and autonomous road survey for PhoenixDrone',
    license='MIT',
    entry_points={
        'console_scripts': [
            'flight_controller = phoenix_drone_control.flight_controller:main',
            'road_survey = phoenix_drone_control.road_survey:main',
            'odom_tf_publisher = phoenix_drone_control.odom_tf_publisher:main',
        ],
    },
)
