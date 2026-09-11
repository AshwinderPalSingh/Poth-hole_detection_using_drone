"""
control.launch.py — Flight controller + autonomous road survey
═══════════════════════════════════════════════════════════════
Usage:
    ros2 launch phoenix_drone_control control.launch.py
    ros2 launch phoenix_drone_control control.launch.py altitude:=8.0
    ros2 launch phoenix_drone_control control.launch.py survey:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    altitude = LaunchConfiguration('altitude')
    survey = LaunchConfiguration('survey')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('altitude', default_value='6.0',
                              description='Cruise altitude in metres AGL'),
        DeclareLaunchArgument('survey', default_value='true',
                              description='Run the autonomous road survey'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ── PID flight controller ──
        Node(
            package='phoenix_drone_control',
            executable='flight_controller',
            name='flight_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'takeoff_altitude': altitude,
                'mass': 1.5,
                'control_rate': 50.0,
                'model_name': 'phoenix_drone',
                'body_name': 'base_link',
                'auto_takeoff': True,
            }],
        ),

        # ── Autonomous lawnmower road survey ──
        Node(
            package='phoenix_drone_control',
            executable='road_survey',
            name='road_survey',
            output='screen',
            condition=IfCondition(survey),
            parameters=[{
                'use_sim_time': use_sim_time,
                'survey_altitude': altitude,
                'road_start_x': 5.0,
                'road_end_x': 95.0,
                'road_width': 6.0,
                'lane_spacing': 3.0,
                'waypoint_spacing': 15.0,
                'arrival_radius': 2.0,
                'update_rate_hz': 5.0,
                'start_delay_s': 14.0,
                'auto_start': True,
            }],
        ),
    ])
