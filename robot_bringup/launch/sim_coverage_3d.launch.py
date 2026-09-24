#!/usr/bin/env python3
"""
Sim Coverage 3D Bringup Launch File
====================================
Shortcut bringup untuk menjalankan 3D Terrain Coverage (Fields2Cover + CMU Autonomy + LIO-SAM) di Gazebo Harmonic.
Dapat dijalankan melalui:
    ros2 launch robot_bringup sim_coverage_3d.launch.py
atau
    ros2 launch robot_navigation coverage_3d.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_navigation = get_package_share_directory('robot_navigation')

    world_arg = DeclareLaunchArgument('world', default_value='outdoor_park.world', description='File world Gazebo Harmonic')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Gunakan clock simulasi')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Buka RViz2 otomatis')
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Posisi spawn robot X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Posisi spawn robot Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.20', description='Posisi spawn robot Z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Rotasi yaw robot (rad)')
    max_speed_arg = DeclareLaunchArgument('max_speed', default_value='0.45', description='Kecepatan maksimum robot (m/s)')
    autonomy_speed_arg = DeclareLaunchArgument('autonomy_speed', default_value='0.35', description='Kecepatan jelajah otonom robot (m/s)')
    cov_width_arg = DeclareLaunchArgument('cov_width', default_value='0.80', description='Lebar sapuan swath coverage (meter)')
    robot_width_arg = DeclareLaunchArgument('robot_width', default_value='0.40', description='Lebar fisik robot (meter)')
    auto_boundary_arg = DeclareLaunchArgument('auto_boundary', default_value='true', description='Otomatis trigger batas padang rumput')
    auto_boundary_delay_arg = DeclareLaunchArgument('auto_boundary_delay', default_value='12.0', description='Detik tunda sebelum kirim auto-boundary')
    bx_min_arg = DeclareLaunchArgument('boundary_x_min', default_value='-7.0', description='Batas area meadow X min')
    bx_max_arg = DeclareLaunchArgument('boundary_x_max', default_value='7.0', description='Batas area meadow X max')
    by_min_arg = DeclareLaunchArgument('boundary_y_min', default_value='-5.0', description='Batas area meadow Y min')
    by_max_arg = DeclareLaunchArgument('boundary_y_max', default_value='5.0', description='Batas area meadow Y max')
    load_pcd_arg = DeclareLaunchArgument('load_pcd', default_value='true', description='Muat peta 3D PCD yang sudah ada ke RViz')
    pcd_path_arg = DeclareLaunchArgument('pcd_path', default_value='/home/akiratanaya/maps/outdoor_park/GlobalMap.pcd', description='Path file 3D PCD map')

    coverage_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_navigation, 'launch', 'coverage_3d.launch.py')
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': LaunchConfiguration('rviz'),
            'load_pcd': LaunchConfiguration('load_pcd'),
            'pcd_path': LaunchConfiguration('pcd_path'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'max_speed': LaunchConfiguration('max_speed'),
            'autonomy_speed': LaunchConfiguration('autonomy_speed'),
            'cov_width': LaunchConfiguration('cov_width'),
            'robot_width': LaunchConfiguration('robot_width'),
            'auto_boundary': LaunchConfiguration('auto_boundary'),
            'auto_boundary_delay': LaunchConfiguration('auto_boundary_delay'),
            'boundary_x_min': LaunchConfiguration('boundary_x_min'),
            'boundary_x_max': LaunchConfiguration('boundary_x_max'),
            'boundary_y_min': LaunchConfiguration('boundary_y_min'),
            'boundary_y_max': LaunchConfiguration('boundary_y_max'),
        }.items()
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        rviz_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        max_speed_arg,
        autonomy_speed_arg,
        cov_width_arg,
        robot_width_arg,
        auto_boundary_arg,
        auto_boundary_delay_arg,
        bx_min_arg,
        bx_max_arg,
        by_min_arg,
        by_max_arg,
        load_pcd_arg,
        pcd_path_arg,
        coverage_3d_launch,
    ])
