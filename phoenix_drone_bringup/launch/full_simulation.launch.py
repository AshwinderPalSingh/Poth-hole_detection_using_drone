#!/usr/bin/env python3
"""
full_simulation.launch.py — Complete PhoenixDrone simulation stack
═══════════════════════════════════════════════════════════════════
Brings up everything in one command:

  Gazebo Classic 11 (pothole_road world)
    └─ PhoenixDrone model (camera + LiDAR + IMU + GPS + odometry)
  Flight controller (PID, auto-takeoff)
    └─ Road survey (autonomous lawnmower pattern)
  Perception (pothole detection → georeferenced map)
  RViz + TF

Usage:
    ros2 launch phoenix_drone_bringup full_simulation.launch.py
    ros2 launch phoenix_drone_bringup full_simulation.launch.py rviz:=true
    ros2 launch phoenix_drone_bringup full_simulation.launch.py perception:=false
    ros2 launch phoenix_drone_bringup full_simulation.launch.py survey:=false gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory('phoenix_drone_description')
    gazebo_pkg = get_package_share_directory('phoenix_drone_gazebo')
    control_pkg = get_package_share_directory('phoenix_drone_control')
    perception_pkg = get_package_share_directory('phoenix_drone_perception')
    bringup_pkg = get_package_share_directory('phoenix_drone_bringup')

    world_file = os.path.join(gazebo_pkg, 'worlds', 'pothole_road.world')
    model_sdf = os.path.join(desc_pkg, 'models', 'phoenix_drone', 'model.sdf')
    urdf_file = os.path.join(desc_pkg, 'urdf', 'phoenix_drone.urdf')
    rviz_config = os.path.join(bringup_pkg, 'config', 'rviz_config.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── Launch configurations ──
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    perception = LaunchConfiguration('perception')
    survey = LaunchConfiguration('survey')
    altitude = LaunchConfiguration('altitude')
    use_yolo = LaunchConfiguration('use_yolo')
    model_path = LaunchConfiguration('model_path')

    model_env = ':'.join([
        os.path.join(desc_pkg, 'models'),
        os.path.join(gazebo_pkg, 'models'),
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    ])

    return LaunchDescription([
        # ══════════ Arguments ══════════
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Show the Gazebo client GUI'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2'),
        DeclareLaunchArgument('perception', default_value='true',
                              description='Run detection + mapping'),
        DeclareLaunchArgument('survey', default_value='true',
                              description='Run the autonomous road survey'),
        DeclareLaunchArgument('altitude', default_value='6.0',
                              description='Cruise altitude, metres AGL'),
        DeclareLaunchArgument('use_yolo', default_value='false',
                              description='Use YOLOv8 instead of classical CV'),
        DeclareLaunchArgument('model_path', default_value='',
                              description='Path to a trained YOLOv8 .pt model'),

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_env),

        # ══════════ Gazebo ══════════
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 '-s', 'libgazebo_ros_force_system.so'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(gui),
        ),

        # ══════════ Robot description / TF ══════════
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ══════════ Spawn the drone ══════════
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_phoenix_drone',
                    arguments=[
                        '-entity', 'phoenix_drone',
                        '-file', model_sdf,
                        '-x', '2.0', '-y', '0.0', '-z', '0.20',
                        '-R', '0', '-P', '0', '-Y', '0',
                    ],
                    output='screen',
                ),
            ],
        ),

        # ══════════ Odometry → TF bridge ══════════
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='phoenix_drone_control',
                    executable='odom_tf_publisher',
                    name='odom_tf_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                ),
            ],
        ),

        # ══════════ Control stack ══════════
        TimerAction(
            period=7.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(control_pkg, 'launch',
                                     'control.launch.py')),
                    launch_arguments={
                        'altitude': altitude,
                        'survey': survey,
                        'use_sim_time': use_sim_time,
                    }.items(),
                ),
            ],
        ),

        # ══════════ Perception stack ══════════
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(perception_pkg, 'launch',
                                     'perception.launch.py')),
                    launch_arguments={
                        'use_yolo': use_yolo,
                        'model_path': model_path,
                        'use_sim_time': use_sim_time,
                    }.items(),
                    condition=IfCondition(perception),
                ),
            ],
        ),

        # ══════════ RViz ══════════
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                    condition=IfCondition(rviz),
                ),
            ],
        ),
    ])
