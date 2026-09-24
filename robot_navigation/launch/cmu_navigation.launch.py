#!/usr/bin/env python3
"""
CMU Navigation Launch File
--------------------------
Menjalankan mesin navigasi CMU Autonomy Stack (3D Terrain Analysis & Local Planner):
1. cmu_sim_bridge: Menjembatani Gazebo /odom dan /points ke /state_estimation dan /registered_scan
2. terrain_analysis: Menghasilkan /terrain_map dari pointcloud 3D
3. terrain_analysis_ext: Analisis konektivitas medan 3D jangkauan luas
4. sensor_scan_generation: Sinkronisasi scan LiDAR
5. local_planner (localPlanner + pathFollower):
   - config: standard (omniDirGoalThre = -1.0 => kinematika roda diferensial)
   - autonomyMode: true (otomatis bergerak mengikuti jalur ke waypoint)
   - twoWayDrive: false (hanya bergerak maju)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg_local_planner = get_package_share_directory('local_planner')
    pkg_terrain_analysis = get_package_share_directory('terrain_analysis')
    pkg_terrain_analysis_ext = get_package_share_directory('terrain_analysis_ext')
    pkg_sensor_scan_gen = get_package_share_directory('sensor_scan_generation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    max_speed = LaunchConfiguration('max_speed')
    autonomy_speed = LaunchConfiguration('autonomy_speed')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_max_speed = DeclareLaunchArgument('max_speed', default_value='0.45')
    declare_autonomy_speed = DeclareLaunchArgument('autonomy_speed', default_value='0.35')

    # 1. CMU Simulation Bridge (Penghubung Gazebo ke CMU)
    cmu_sim_bridge_node = Node(
        package='robot_navigation',
        executable='cmu_sim_bridge.py',
        name='cmu_sim_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. Terrain Analysis (Menganalisis tanah & rintangan 3D)
    terrain_analysis_node = Node(
        package='terrain_analysis',
        executable='terrainAnalysis',
        name='terrainAnalysis',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scanVoxelSize': 0.05,
            'decayTime': 1.0,
            'noDecayDis': 1.75,
            'clearingDis': 8.0,
            'useSorting': True,
            'quantileZ': 0.25,
            'considerDrop': False,
            'limitGroundLift': False,
            'maxGroundLift': 0.15,
            'clearDyObs': True,
            'sensorPitch': 0.0,              # LiDAR horizontal rata (bukan miring 20 derajat)
            'minDyObsDis': 0.14,
            'absDyObsRelZThre': 0.2,
            'minDyObsVFOV': -10.0,
            'maxDyObsVFOV': 55.0,
            'minDyObsPointNum': 1,
            'minOutOfFovPointNum': 10,
            'obstacleHeightThre': 0.09,
            'nearObstacle': False,            # Nonaktifkan agar lantai di sekitar robot tidak dicap rintangan
            'nearObstacleDis': 0.0,
            'nearObstacleRelZThre': -0.3,
            'negObstacle': -1,
            'negObstacleDis': 10.0,
            'negObstacleRelZThre': -0.2,
            'noDataObstacle': False,
            'vehicleHeight': 1.5,
            'minRelZ': -1.5,
            'maxRelZ': 0.3,
            'disRatioZ': 0.2,
        }]
    )

    # 3. Terrain Analysis Ext (Jangkauan terrain luas)
    terrain_analysis_ext_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_terrain_analysis_ext, 'launch', 'terrain_analysis_ext.launch')
        ),
        launch_arguments={'checkTerrainConn': 'true'}.items()
    )

    # 4. Sensor Scan Generation (Sinkronisasi frame sensor)
    sensor_scan_gen_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_sensor_scan_gen, 'launch', 'sensor_scan_generation.launch')
        )
    )

    # 5. Local Planner & Path Follower (Mode Differential Drive)
    local_planner_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_local_planner, 'launch', 'local_planner.launch')
        ),
        launch_arguments={
            'config': 'standard',
            'realRobot': 'false',
            'autonomyMode': 'true',
            'twoWayDrive': 'true',
            'waitForWaypoint': 'true',
            'maxSpeed': max_speed,
            'autonomySpeed': autonomy_speed,
            'sensorOffsetX': '0.0',
            'sensorOffsetY': '0.0',
            'cameraOffsetZ': '0.0',
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_max_speed,
        declare_autonomy_speed,
        SetParameter(name='use_sim_time', value=use_sim_time),
        cmu_sim_bridge_node,
        terrain_analysis_node,
        terrain_analysis_ext_launch,
        sensor_scan_gen_launch,
        local_planner_launch,
    ])
