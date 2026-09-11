"""
simulation.launch.py — Gazebo world + PhoenixDrone only
════════════════════════════════════════════════════════
Starts Gazebo Classic with the pothole_road world and spawns the
PhoenixDrone model.  No control or perception — use this to inspect
the world, or as a building block for the bringup launch.

Usage:
    ros2 launch phoenix_drone_gazebo simulation.launch.py
    ros2 launch phoenix_drone_gazebo simulation.launch.py gui:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('phoenix_drone_gazebo')
    desc_pkg = get_package_share_directory('phoenix_drone_description')

    default_world = os.path.join(gazebo_pkg, 'worlds', 'pothole_road.world')
    model_sdf = os.path.join(desc_pkg, 'models', 'phoenix_drone', 'model.sdf')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    x, y, z = (LaunchConfiguration('x'),
               LaunchConfiguration('y'),
               LaunchConfiguration('z'))

    # Make both packages' models resolvable by model:// URIs
    model_path = ':'.join([
        os.path.join(desc_pkg, 'models'),
        os.path.join(gazebo_pkg, 'models'),
        os.environ.get('GAZEBO_MODEL_PATH', ''),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Path to the Gazebo world file'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start the Gazebo client GUI'),
        DeclareLaunchArgument('x', default_value='2.0',
                              description='Drone spawn X'),
        DeclareLaunchArgument('y', default_value='0.0',
                              description='Drone spawn Y'),
        DeclareLaunchArgument('z', default_value='0.20',
                              description='Drone spawn Z'),

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),

        # ── Gazebo server ──
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 '-s', 'libgazebo_ros_force_system.so'],
            output='screen',
        ),

        # ── Gazebo client (optional) ──
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            condition=IfCondition(gui),
        ),

        # ── Spawn the drone once the server is up ──
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
                        '-x', x, '-y', y, '-z', z,
                        '-R', '0', '-P', '0', '-Y', '0',
                    ],
                    output='screen',
                ),
            ],
        ),
    ])
