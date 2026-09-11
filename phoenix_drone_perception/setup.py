from setuptools import setup
import os
from glob import glob

package_name = 'phoenix_drone_perception'

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
        (os.path.join('share', package_name, 'models'),
            glob('models/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PhoenixDrone Dev',
    maintainer_email='dev@phoenix-drone.local',
    description='YOLOv8 pothole detection and geo-mapping for PhoenixDrone',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pothole_detector = phoenix_drone_perception.pothole_detector:main',
            'pothole_mapper = phoenix_drone_perception.pothole_mapper:main',
            'navigation_monitor = phoenix_drone_perception.navigation_monitor:main',
        ],
    },
)
