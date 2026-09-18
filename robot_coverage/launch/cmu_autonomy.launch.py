#!/usr/bin/env python3
"""
CMU Autonomy Stack Launch File
------------------------------
Menjalankan seluruh modul navigasi CMU (differential drive) + Fields2Cover:
1. cmu_sim_bridge: Menjembatani Gazebo /odom dan /points ke /state_estimation dan /registered_scan
2. terrain_analysis: Menghasilkan /terrain_map dari pointcloud 3D
3. terrain_analysis_ext: Analisis konektivitas medan 3D
4. sensor_scan_generation: Sinkronisasi sensor scan
5. local_planner (localPlanner + pathFollower):
   - config: standard (omniDirGoalThre = -1.0 => roda diferensial)
   - autonomyMode: true (otomatis bergerak ke /way_point)
   - twoWayDrive: false (hanya gerak maju menghadap target)
6. coverage_server & field_boundary_collector: Generator coverage Fields2Cover
7. cmu_coverage_navigator: Menghubungkan /coverage_path ke /way_point
8. footprint_trail_visualizer: Menampilkan jejak visual area yang sudah disapu
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_coverage = get_package_share_directory('robot_coverage')
    pkg_local_planner = get_package_share_directory('local_planner')
    pkg_terrain_analysis = get_package_share_directory('terrain_analysis')
    pkg_terrain_analysis_ext = get_package_share_directory('terrain_analysis_ext')
    pkg_sensor_scan_gen = get_package_share_directory('sensor_scan_generation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    max_speed = LaunchConfiguration('max_speed')
    autonomy_speed = LaunchConfiguration('autonomy_speed')
    waypoint_tolerance = LaunchConfiguration('waypoint_tolerance')
    auto_navigate = LaunchConfiguration('auto_navigate')
    decomp_method = LaunchConfiguration('decomp_method')
    route_pattern = LaunchConfiguration('route_pattern')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_max_speed = DeclareLaunchArgument('max_speed', default_value='0.6')
    declare_autonomy_speed = DeclareLaunchArgument('autonomy_speed', default_value='0.6')
    declare_waypoint_tolerance = DeclareLaunchArgument('waypoint_tolerance', default_value='0.35')
    declare_auto_navigate = DeclareLaunchArgument('auto_navigate', default_value='true')
    declare_decomp_method = DeclareLaunchArgument('decomp_method', default_value='boustrophedon')
    declare_route_pattern = DeclareLaunchArgument('route_pattern', default_value='boustrophedon')

    # 1. CMU Sim Bridge
    cmu_sim_bridge_node = Node(
        package='robot_coverage',
        executable='cmu_sim_bridge',
        name='cmu_sim_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. Terrain Analysis
    terrain_analysis_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_terrain_analysis, 'launch', 'terrain_analysis.launch')
        )
    )

    # 3. Terrain Analysis Ext
    terrain_analysis_ext_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_terrain_analysis_ext, 'launch', 'terrain_analysis_ext.launch')
        ),
        launch_arguments={'checkTerrainConn': 'true'}.items()
    )

    # 4. Sensor Scan Generation
    sensor_scan_gen_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_sensor_scan_gen, 'launch', 'sensor_scan_generation.launch')
        )
    )

    # 5. Local Planner (config: standard untuk differential drive)
    local_planner_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_local_planner, 'launch', 'local_planner.launch')
        ),
        launch_arguments={
            'config': 'standard',
            'realRobot': 'false',
            'autonomyMode': 'true',
            'twoWayDrive': 'false',
            'maxSpeed': max_speed,
            'autonomySpeed': autonomy_speed,
            'sensorOffsetX': '0.0',
            'sensorOffsetY': '0.0',
            'cameraOffsetZ': '0.0',
        }.items()
    )

    # 6. Fields2Cover Coverage Planner Server
    coverage_server_node = Node(
        package='robot_coverage',
        executable='coverage_server',
        name='coverage_server',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_compute': True,
            'decomp_method': decomp_method,
            'route_pattern': route_pattern,
        }],
        output='screen'
    )

    # 7. Field Boundary Collector
    field_boundary_collector_node = Node(
        package='robot_coverage',
        executable='field_boundary_collector',
        name='field_boundary_collector',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 8. CMU Coverage Navigator
    cmu_coverage_navigator_node = Node(
        package='robot_coverage',
        executable='cmu_coverage_navigator',
        name='cmu_coverage_navigator',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoint_tolerance': waypoint_tolerance,
            'auto_navigate': auto_navigate,
        }],
        output='screen'
    )

    # 9. Swept Footprint Visualizer
    footprint_trail_node = Node(
        package='robot_coverage',
        executable='footprint_trail_visualizer',
        name='footprint_trail_visualizer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_max_speed,
        declare_autonomy_speed,
        declare_waypoint_tolerance,
        declare_auto_navigate,
        declare_decomp_method,
        declare_route_pattern,
        cmu_sim_bridge_node,
        terrain_analysis_launch,
        terrain_analysis_ext_launch,
        sensor_scan_gen_launch,
        local_planner_launch,
        coverage_server_node,
        field_boundary_collector_node,
        cmu_coverage_navigator_node,
        footprint_trail_node,
    ])
