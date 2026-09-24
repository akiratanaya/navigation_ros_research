#!/usr/bin/env python3
"""
Sim Auto Mapping 3D Bringup Launch File
=======================================
Shortcut bringup untuk menjalankan 3D Auto-Mapping (LIO-SAM + CMU Autonomy Stack) di Gazebo Harmonic.
Dapat dijalankan melalui:
    ros2 launch robot_bringup sim_auto_mapping_3d.launch.py
atau
    ros2 launch robot_navigation auto_mapping_3d.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_navigation = get_package_share_directory('robot_navigation')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='outdoor_park.world',
        description='Nama file world Gazebo (default: outdoor_park.world)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gunakan simulasi clock Gazebo jika true'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Buka RViz2 visualisasi 3D auto mapping otomatis'
    )
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='false',
        description='Buka terminal pop-up Teleop Keyboard manual override jika true'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Posisi spawn robot X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Posisi spawn robot Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.15', description='Posisi spawn robot Z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Rotasi yaw robot saat spawn (rad)')
    max_speed_arg = DeclareLaunchArgument('max_speed', default_value='0.45', description='Kecepatan maksimum robot (m/s)')
    autonomy_speed_arg = DeclareLaunchArgument('autonomy_speed', default_value='0.35', description='Kecepatan jelajah otonom robot (m/s)')
    boundary_size_arg = DeclareLaunchArgument('boundary_size', default_value='42.0', description='Radius batas area penjelajahan taman (meter)')

    auto_mapping_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_navigation, 'launch', 'auto_mapping_3d.launch.py')
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': LaunchConfiguration('rviz'),
            'teleop': LaunchConfiguration('teleop'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'max_speed': LaunchConfiguration('max_speed'),
            'autonomy_speed': LaunchConfiguration('autonomy_speed'),
            'boundary_size': LaunchConfiguration('boundary_size'),
        }.items()
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        rviz_arg,
        teleop_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        max_speed_arg,
        autonomy_speed_arg,
        boundary_size_arg,
        auto_mapping_3d_launch
    ])
