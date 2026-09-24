#!/usr/bin/env python3
"""
3D Autonomous Mapping & Exploration Launch File
================================================
Integrasi lengkap LIO-SAM 3D SLAM + CMU Autonomy Stack (TARE Planner + Terrain Analysis + Local Planner).
Robot melakukan penjelajahan dan pemetaan 3D secara penuh otonom di dunia berkontur (outdoor_park.world).

Komponen:
1. Gazebo Harmonic Simulator: Memuat outdoor_park.world dengan Velodyne VLP-16 (Mode 3D) & IMU
2. LIO-SAM 3D SLAM: Menghasilkan odometri IMU preintegration presisi tinggi & peta awan titik 3D
3. LioSam-CMU Bridge: Menjembatani odometri, registrasi scan 3D, batas eksplorasi, & memicu start eksplorasi
4. CMU Terrain Analysis: Analisis elevasi permukaan tanah dan rintangan 3D
5. CMU Sensor Scan Generation: Sinkronisasi scan sensor dan odometri
6. CMU Local Planner & Path Follower: Perencana gerak lokal dan pengontrol kecepatan /cmd_vel
7. CMU TARE Planner: Perencana eksplorasi global 3D berbasis Hierarchical TSP & Frontier Exploration
8. RViz2: Visualisasi terpadu SLAM, Medan Tanah, Titik Frontier, dan Jalur Robot
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess, OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def launch_setup(context, *args, **kwargs):
    # Package directories
    pkg_description = get_package_share_directory('robot_description')
    pkg_navigation = get_package_share_directory('robot_navigation')
    pkg_lio_sam = get_package_share_directory('lio_sam')
    pkg_local_planner = get_package_share_directory('local_planner')
    pkg_terrain_analysis = get_package_share_directory('terrain_analysis')
    pkg_terrain_analysis_ext = get_package_share_directory('terrain_analysis_ext')
    pkg_sensor_scan_gen = get_package_share_directory('sensor_scan_generation')
    pkg_tare_planner = get_package_share_directory('tare_planner')

    # Launch Configurations
    world_val = context.launch_configurations.get('world', 'outdoor_park.world')
    x_val = context.launch_configurations.get('x', '0.0')
    y_val = context.launch_configurations.get('y', '0.0')
    z_val = context.launch_configurations.get('z', '0.15')
    yaw_val = context.launch_configurations.get('yaw', '0.0')
    max_speed_val = float(context.launch_configurations.get('max_speed', '0.45'))
    autonomy_speed_val = float(context.launch_configurations.get('autonomy_speed', '0.35'))
    boundary_size_val = float(context.launch_configurations.get('boundary_size', '42.0'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_rviz = LaunchConfiguration('rviz')
    run_teleop = LaunchConfiguration('teleop')

    # Path file konfigurasi
    rviz_config = os.path.join(pkg_navigation, 'rviz', 'auto_mapping_3d.rviz')
    tare_config = os.path.join(pkg_tare_planner, 'outdoor.yaml')
    local_planner_config = os.path.join(pkg_local_planner, 'config', 'standard.yaml')

    # =========================================================================
    # 1. Gazebo Harmonic Simulation (Taman Berbukit 3D + Velodyne VLP-16)
    # =========================================================================
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'sim.launch.py')]),
        launch_arguments={
            'world': world_val,
            'x': x_val,
            'y': y_val,
            'z': z_val,
            'yaw': yaw_val,
            'lidar_mode': '3d',           # Mode 3D PointCloud Velodyne
            'bridge_tf': 'false',         # LIO-SAM yang mempublikasikan odom TF (mencegah bentrok TF)
            'use_sim_time': use_sim_time,
            'rviz_sim': 'false'           # Menggunakan RViz gabungan kita
        }.items()
    )

    # =========================================================================
    # 2. LIO-SAM 3D SLAM (Delay 3.0 detik agar Gazebo & topik sensor stabil)
    # =========================================================================
    lio_sam_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_lio_sam, 'launch', 'run.launch.py')]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'rviz': 'false',              # Visualisasi via auto_mapping_3d.rviz
                    'publish_robot_state': 'false'  # Sudah dipublikasikan oleh sim.launch.py
                }.items()
            )
        ]
    )

    # =========================================================================
    # 3. LIO-SAM to CMU Bridge (Delay 4.0 detik)
    #    Mengubah /odometry/imu -> /state_estimation dan /points -> /registered_scan
    # =========================================================================
    bridge_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='robot_navigation',
                executable='lio_sam_cmu_bridge.py',
                name='lio_sam_cmu_bridge',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'world_frame': 'map',
                    'robot_base_frame': 'base_footprint',
                    'lidar_frame': 'lidar_3d_link',
                    'robot_radius': 0.28,
                    'boundary_size': boundary_size_val,
                }]
            )
        ]
    )

    # =========================================================================
    # 4. CMU Terrain Analysis & Sensor Scan Generation (Delay 4.5 detik)
    # =========================================================================
    cmu_terrain_nodes = TimerAction(
        period=4.5,
        actions=[
            # 4a. Terrain Analysis (Klasifikasi permukaan tanah & rintangan)
            Node(
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
                    'sensorPitch': 0.0,
                    'minDyObsDis': 0.14,
                    'absDyObsRelZThre': 0.2,
                    'minDyObsVFOV': -10.0,
                    'maxDyObsVFOV': 55.0,
                    'minDyObsPointNum': 1,
                    'minOutOfFovPointNum': 10,
                    'obstacleHeightThre': 0.09,
                    'nearObstacle': False,
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
            ),
            # 4b. Terrain Analysis Ext (Analisis konektivitas medan 3D skala luas)
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(
                    pkg_terrain_analysis_ext, 'launch', 'terrain_analysis_ext.launch')
                ),
                launch_arguments={'checkTerrainConn': 'true'}.items()
            ),
            # 4c. Sensor Scan Generation (Sinkronisasi scan LiDAR dan odometri)
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(
                    pkg_sensor_scan_gen, 'launch', 'sensor_scan_generation.launch')
                )
            ),
        ]
    )

    # =========================================================================
    # 5. CMU Local Planner & Path Follower (Delay 5.0 detik)
    #    Mengendalikan navigasi lokal robot dan mengeluarkan /cmd_vel
    # =========================================================================
    cmu_local_planner_nodes = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='local_planner',
                executable='localPlanner',
                name='localPlanner',
                output='screen',
                parameters=[
                    local_planner_config,
                    {
                        'use_sim_time': use_sim_time,
                        'pathFolder': os.path.join(pkg_local_planner, 'paths'),
                        'vehicleLength': 0.4,
                        'vehicleWidth': 0.4,
                        'sensorOffsetX': 0.0,
                        'sensorOffsetY': 0.0,
                        'vehicleLengthSlot': 0.05,
                        'vehicleWidthMargin': 0.12,
                        'marginYawRateRatio': 0.0,
                        'twoWayDrive': True,
                        'laserVoxelSize': 0.05,
                        'terrainVoxelSize': 0.2,
                        'useTerrainAnalysis': True,
                        'checkObstacle': True,
                        'checkRotObstacle': True,
                        'adjacentRange': 3.5,
                        'obstacleHeightThre': 0.09,
                        'groundHeightThre': 0.05,
                        'costHeightThre1': 0.25,
                        'costHeightThre2': 0.15,
                        'useCost': False,
                        'slowPathNumThre': 5,
                        'slowGroupNumThre': 1,
                        'surPointThre': 5,
                        'pointPerPathThre': 6,
                        'minRelZ': -0.4,
                        'maxRelZ': 0.3,
                        'maxSpeed': max_speed_val,
                        'dirWeight': 0.02,
                        'dirThre': 90.0,
                        'dirToVehicle': False,
                        'pathScale': 0.875,
                        'minPathScale': 0.675,
                        'pathScaleStep': 0.1,
                        'pathScaleBySpeed': True,
                        'minPathRange': 0.8,
                        'pathRangeStep': 0.6,
                        'pathRangeBySpeed': True,
                        'pathCropByGoal': True,
                        'autonomyMode': True,
                        'autonomySpeed': autonomy_speed_val,
                        'joyToSpeedDelay': 2.0,
                        'joyToCheckObstacleDelay': 5.0,
                        'goalClearRange': 0.35,
                        'goalBehindRange': 0.35,
                        'freezeAng': 90.0,
                        'freezeTime': 0.0,
                        'goalX': float(x_val),
                        'goalY': float(y_val),
                        'waitForWaypoint': True,
                    }
                ]
            ),
            Node(
                package='local_planner',
                executable='pathFollower',
                name='pathFollower',
                output='screen',
                parameters=[
                    local_planner_config,
                    {
                        'use_sim_time': use_sim_time,
                        'realRobot': False,
                        'serialPort': '/dev/ttyACM0',
                        'baudrate': 115200,
                        'sensorOffsetX': 0.0,
                        'sensorOffsetY': 0.0,
                        'pubSkipNum': 1,
                        'twoWayDrive': True,
                        'lookAheadDis': 0.5,
                        'maxSpeed': max_speed_val,
                        'maxAccel': 2.0,
                        'switchTimeThre': 1.0,
                        'omniDirDiffThre': 1.5,
                        'slowDwnDisThre': 0.875,
                        'useInclRateToSlow': False,
                        'inclRateThre': 120.0,
                        'slowRate1': 0.25,
                        'slowRate2': 0.5,
                        'slowRate3': 0.75,
                        'slowTime1': 2.0,
                        'slowTime2': 2.0,
                        'useSideAvoid': False,
                        'useInclToStop': False,
                        'inclThre': 45.0,
                        'stopTime': 5.0,
                        'noRotAtStop': False,
                        'noRotAtGoal': True,
                        'autonomyMode': True,
                        'autonomySpeed': autonomy_speed_val,
                        'joyToSpeedDelay': 2.0,
                    }
                ]
            )
        ]
    )

    # =========================================================================
    # 6. CMU TARE Planner (Delay 6.0 detik)
    #    Hierarchical TSP exploration algorithm untuk pemetaan 3D otomatis
    # =========================================================================
    tare_or_tools_lib = '/home/akiratanaya/robotics/delabo_itb/navigation_ros_ws/src/cmu_autonomy_stack/src/exploration_planner/tare_planner/or-tools/lib'
    tare_planner_action = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='tare_planner',
                executable='tare_planner_node',
                name='tare_planner_node',
                output='screen',
                parameters=[tare_config],
                additional_env={
                    'LD_LIBRARY_PATH': f"{tare_or_tools_lib}:" + os.environ.get('LD_LIBRARY_PATH', '')
                }
            )
        ]
    )

    # =========================================================================
    # 7. RViz2 Visualizer (Delay 5.5 detik)
    # =========================================================================
    rviz_action = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_auto_mapping_3d',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(run_rviz),
                output='screen'
            )
        ]
    )

    # =========================================================================
    # 8. Optional Teleop Keyboard (Manual override bila dibutuhkan)
    # =========================================================================
    teleop_action = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'xterm',
                    '-title', 'AutoNav Bot - Teleop Keyboard Override',
                    '-geometry', '75x22',
                    '-fa', 'Monospace',
                    '-fs', '11',
                    '-bg', '#1e1e2e',
                    '-fg', '#cdd6f4',
                    '-e', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'
                ],
                condition=IfCondition(run_teleop),
                output='screen'
            )
        ]
    )

    return [
        sim_launch,
        lio_sam_launch,
        bridge_node,
        cmu_terrain_nodes,
        cmu_local_planner_nodes,
        tare_planner_action,
        rviz_action,
        teleop_action,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='outdoor_park.world',
            description='Nama file world Gazebo (default: outdoor_park.world)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gunakan simulasi clock Gazebo jika true'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Buka RViz2 visualisasi 3D auto mapping otomatis'
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Buka terminal pop-up Teleop Keyboard manual override jika true'
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Posisi spawn robot X'),
        DeclareLaunchArgument('y', default_value='0.0', description='Posisi spawn robot Y'),
        DeclareLaunchArgument('z', default_value='0.15', description='Posisi spawn robot Z'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Rotasi yaw robot saat spawn (rad)'),
        DeclareLaunchArgument('max_speed', default_value='0.45', description='Kecepatan maksimum robot (m/s)'),
        DeclareLaunchArgument('autonomy_speed', default_value='0.35', description='Kecepatan jelajah otonom robot (m/s)'),
        DeclareLaunchArgument('boundary_size', default_value='42.0', description='Radius batas area penjelajahan taman (meter)'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        OpaqueFunction(function=launch_setup)
    ])
