"""
perception.launch.py — Pothole detection, mapping and telemetry
════════════════════════════════════════════════════════════════
Usage:
    ros2 launch phoenix_drone_perception perception.launch.py
    ros2 launch phoenix_drone_perception perception.launch.py model_path:=/path/to/best.pt
    ros2 launch phoenix_drone_perception perception.launch.py use_yolo:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    use_yolo = LaunchConfiguration('use_yolo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    output_file = LaunchConfiguration('output_file')
    monitor = LaunchConfiguration('monitor')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path', default_value='',
            description='Path to a trained YOLOv8 pothole .pt model. '
                        'Empty uses the classical-CV detector.'),
        DeclareLaunchArgument(
            'use_yolo', default_value='false',
            description='Use YOLOv8 instead of the classical CV detector'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'output_file', default_value='/tmp/pothole_map.json',
            description='Where to write the georeferenced pothole map'),
        DeclareLaunchArgument(
            'monitor', default_value='true',
            description='Run the navigation telemetry monitor'),

        # ── Pothole detector ──
        Node(
            package='phoenix_drone_perception',
            executable='pothole_detector',
            name='pothole_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_path': model_path,
                'use_yolo': use_yolo,
                'camera_topic': '/phoenix/camera/image_raw',
                'confidence_threshold': 0.35,
                'max_inference_fps': 6.0,
                'publish_annotated': True,
                'min_area_px': 150,
                'min_circularity': 0.45,
                'darkness_ratio': 0.35,
            }],
        ),

        # ── Georeferenced mapper ──
        Node(
            package='phoenix_drone_perception',
            executable='pothole_mapper',
            name='pothole_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'output_file': output_file,
                'report_file': '/tmp/pothole_report.txt',
                'min_separation_m': 1.5,
                'min_confidence': 0.35,
                'min_sightings_to_confirm': 3,
                'ref_latitude': 28.6139,
                'ref_longitude': 77.2090,
                'ref_altitude': 216.0,
                'road_plane_z': 0.02,
                'publish_rate_hz': 1.0,
            }],
        ),

        # ── Navigation telemetry monitor ──
        Node(
            package='phoenix_drone_perception',
            executable='navigation_monitor',
            name='navigation_monitor',
            output='screen',
            condition=IfCondition(monitor),
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_rate_hz': 2.0,
            }],
        ),
    ])
