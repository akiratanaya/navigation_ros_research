#!/usr/bin/env python3
"""
3D Terrain Coverage Path Planning Launch File (Fields2Cover + CMU + LIO-SAM)
=============================================================================
Sistem Navigasi & Penyisiran Lahan Berkontur 3D Terpadu:
1. Gazebo Harmonic: Memuat outdoor_park.world (lereng berbukit, rumput, pohon) + LiDAR Velodyne 3D
2. LIO-SAM 3D SLAM: Lokalisasi 6-DoF dan odometri IMU preintegration
3. LioSam-CMU Bridge: Menjembatani odometri (/state_estimation) dan pointcloud (/registered_scan)
4. CMU Terrain Analysis: Menganalisis kecuraman medan 3D dan rintangan secara real-time
5. CMU Local Planner & Path Follower: Mengendalikan /cmd_vel mengikuti kontur medan 3D
6. Fields2Cover Coverage Server: Generator swath optimal, headland, dan belokan Dubins
7. CMU Coverage Bridge: Pure-pursuit lookahead waypoint feeder dari /coverage_path ke /way_point
8. Footprint Trail Visualizer: Menampilkan jejak sapuan 3D dan persentase cakupan real-time
9. RViz2: Visualisasi terpadu SLAM, Medan 3D, Jalur Sapuan, Waypoint Aktif, dan Jejak Robot
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def launch_setup(context, *args, **kwargs):
    # Package directories
    pkg_description = get_package_share_directory('robot_description')
    pkg_navigation = get_package_share_directory('robot_navigation')
    pkg_coverage = get_package_share_directory('robot_coverage')
    pkg_lio_sam = get_package_share_directory('lio_sam')
    pkg_local_planner = get_package_share_directory('local_planner')
    pkg_terrain_analysis = get_package_share_directory('terrain_analysis')
    pkg_terrain_analysis_ext = get_package_share_directory('terrain_analysis_ext')
    pkg_sensor_scan_gen = get_package_share_directory('sensor_scan_generation')

    # Launch Configurations
    world_val = context.launch_configurations.get('world', 'outdoor_park.world')
    x_val = context.launch_configurations.get('x', '0.0')
    y_val = context.launch_configurations.get('y', '0.0')
    z_val = context.launch_configurations.get('z', '0.20')
    yaw_val = context.launch_configurations.get('yaw', '0.0')
    max_speed_val = float(context.launch_configurations.get('max_speed', '0.45'))
    autonomy_speed_val = float(context.launch_configurations.get('autonomy_speed', '0.35'))
    cov_width_val = float(context.launch_configurations.get('cov_width', '0.80'))
    robot_width_val = float(context.launch_configurations.get('robot_width', '0.40'))
    auto_boundary_val = context.launch_configurations.get('auto_boundary', 'true').lower() == 'true'
    auto_boundary_delay_val = float(context.launch_configurations.get('auto_boundary_delay', '12.0'))
    bx_min_val = float(context.launch_configurations.get('boundary_x_min', '-7.0'))
    bx_max_val = float(context.launch_configurations.get('boundary_x_max', '7.0'))
    by_min_val = float(context.launch_configurations.get('boundary_y_min', '-5.0'))
    by_max_val = float(context.launch_configurations.get('boundary_y_max', '5.0'))
    load_pcd_val = context.launch_configurations.get('load_pcd', 'true').lower() == 'true'
    pcd_path_val = context.launch_configurations.get('pcd_path', '/home/akiratanaya/maps/outdoor_park/GlobalMap.pcd')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_rviz = LaunchConfiguration('rviz')
    load_pcd = LaunchConfiguration('load_pcd')

    # Config files
    rviz_config = os.path.join(pkg_navigation, 'rviz', 'coverage_3d.rviz')
    local_planner_config = os.path.join(pkg_local_planner, 'config', 'standard.yaml')

    # Environment untuk Fields2Cover Python SWIG wrapper & library loader
    f2c_env = {
        'PYTHONPATH': '/home/akiratanaya/Fields2Cover/build/swig/python:' + os.environ.get('PYTHONPATH', ''),
        'LD_LIBRARY_PATH': '/opt/ros/jazzy/opt/ortools_vendor/lib:/usr/local/lib:' + os.environ.get('LD_LIBRARY_PATH', ''),
    }

    # =========================================================================
    # 0. Pre-loaded 3D PCD Map Publisher (Peta 3D Langsung Muncul di RViz)
    # =========================================================================
    pcd_map_node = Node(
        package='pcl_ros',
        executable='pcd_to_pointcloud',
        name='pcd_publisher',
        output='screen',
        parameters=[{
            'file_name': pcd_path_val,
            'tf_frame': 'map',
            'publishing_period_ms': 0,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(load_pcd),
    )

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
            'lidar_mode': '3d',
            'bridge_tf': 'false',         # LIO-SAM yang mempublikasikan odom TF
            'use_sim_time': use_sim_time,
            'rviz_sim': 'false',
        }.items()
    )

    # =========================================================================
    # 2. LIO-SAM 3D SLAM (Delay 3.0 detik agar Gazebo & sensor stabil)
    # =========================================================================
    lio_sam_launch = TimerAction(
        period=4.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_lio_sam, 'launch', 'run.launch.py')]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'rviz': 'false',
                    'publish_robot_state': 'false',
                }.items()
            )
        ]
    )

    # =========================================================================
    # 3. LIO-SAM to CMU Bridge (Delay 5.5 detik)
    # =========================================================================
    bridge_node = TimerAction(
        period=5.5,
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
                    'boundary_size': 42.0,
                    'enable_clicked_point': False,
                }]
            )
        ]
    )

    # =========================================================================
    # 4. CMU Terrain Analysis & Sensor Scan Generation (Delay 6.0 detik)
    # =========================================================================
    cmu_terrain_nodes = TimerAction(
        period=6.0,
        actions=[
            # 4a. Terrain Analysis (Analisis kemiringan tanah & rintangan 3D)
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
            # 4b. Terrain Analysis Ext (Konektivitas medan skala luas)
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(
                    pkg_terrain_analysis_ext, 'launch', 'terrain_analysis_ext.launch')
                ),
                launch_arguments={'checkTerrainConn': 'true'}.items()
            ),
            # 4c. Sensor Scan Generation (Sinkronisasi scan LiDAR)
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(os.path.join(
                    pkg_sensor_scan_gen, 'launch', 'sensor_scan_generation.launch')
                )
            ),
        ]
    )

    # =========================================================================
    # 5. CMU Local Planner & Path Follower (Delay 6.5 detik)
    # =========================================================================
    cmu_local_planner_nodes = TimerAction(
        period=6.5,
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
                        'pubSkipNum': 1,
                        'twoWayDrive': True,
                        'lookAheadDis': 0.4,
                        'yawRateMode': True,
                        'maxSpeed': max_speed_val,
                        'autonomyMode': True,
                        'autonomySpeed': autonomy_speed_val,
                        'joyToSpeedDelay': 2.0,
                    }
                ]
            ),
        ]
    )

    # =========================================================================
    # 5b. Field Boundary Collector (Mengumpulkan titik klik 'Publish Point' RViz)
    # =========================================================================
    field_boundary_collector_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='robot_coverage',
                executable='field_boundary_collector',
                name='field_boundary_collector',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    # =========================================================================
    # 6. Fields2Cover Coverage Server (Delay 7.5 detik)
    # =========================================================================
    coverage_server_node = TimerAction(
        period=7.5,
        actions=[
            Node(
                package='robot_coverage',
                executable='coverage_server',
                name='coverage_server',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'auto_compute': True,
                    'robot_width': robot_width_val,
                    'cov_width': cov_width_val,
                    'turning_radius': 0.05,
                    'swath_angle': 0.0,            # 0.0 = Optimasi otomatis arah swath
                    'enable_perimeter_tour': False, # Di padang rumput terbuka, infill swaths paling efisien
                    'headland_swaths': 0,
                }],
                additional_env=f2c_env,
            )
        ]
    )

    # =========================================================================
    # 7. CMU Coverage Bridge (Delay 8.0 detik)
    #    Menghubungkan /coverage_path ke /way_point CMU Local Planner
    # =========================================================================
    cmu_coverage_bridge_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='robot_navigation',
                executable='cmu_coverage_bridge.py',
                name='cmu_coverage_bridge',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'reach_threshold': 0.65,
                    'lookahead_dist': 1.20,
                    'waypoint_rate': 10.0,
                    'auto_boundary': auto_boundary_val,
                    'auto_boundary_delay': auto_boundary_delay_val,
                    'boundary_x_min': bx_min_val,
                    'boundary_x_max': bx_max_val,
                    'boundary_y_min': by_min_val,
                    'boundary_y_max': by_max_val,
                }]
            )
        ]
    )

    # =========================================================================
    # 8. Real-time Footprint Trail Visualizer (Delay 8.5 detik)
    # =========================================================================
    footprint_trail_node = TimerAction(
        period=8.5,
        actions=[
            Node(
                package='robot_coverage',
                executable='footprint_trail_visualizer',
                name='footprint_trail_visualizer',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'tool_width': cov_width_val,
                    'update_dist': 0.04,
                    'resolution': 0.05,
                }]
            )
        ]
    )

    # =========================================================================
    # 9. RViz2 Terpadu (Delay 3.0 detik)
    # =========================================================================
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_coverage_3d',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(run_rviz),
                output='screen',
            )
        ]
    )

    return [
        pcd_map_node,
        sim_launch,
        lio_sam_launch,
        bridge_node,
        cmu_terrain_nodes,
        cmu_local_planner_nodes,
        field_boundary_collector_node,
        coverage_server_node,
        cmu_coverage_bridge_node,
        footprint_trail_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='outdoor_park.world', description='File world Gazebo Harmonic'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Gunakan clock simulasi'),
        DeclareLaunchArgument('rviz', default_value='true', description='Buka RViz2 otomatis'),
        DeclareLaunchArgument('load_pcd', default_value='true', description='Muat peta 3D PCD yang sudah ada ke RViz'),
        DeclareLaunchArgument('pcd_path', default_value='/home/akiratanaya/maps/outdoor_park/GlobalMap.pcd', description='Path file 3D PCD map'),
        DeclareLaunchArgument('x', default_value='0.0', description='Posisi spawn robot X'),
        DeclareLaunchArgument('y', default_value='0.0', description='Posisi spawn robot Y'),
        DeclareLaunchArgument('z', default_value='0.15', description='Posisi spawn robot Z'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Rotasi yaw spawn (rad)'),
        DeclareLaunchArgument('max_speed', default_value='0.45', description='Kecepatan maksimum robot (m/s)'),
        DeclareLaunchArgument('autonomy_speed', default_value='0.35', description='Kecepatan jelajah otonom robot (m/s)'),
        DeclareLaunchArgument('cov_width', default_value='0.80', description='Lebar sapuan swath coverage (meter)'),
        DeclareLaunchArgument('robot_width', default_value='0.40', description='Lebar fisik robot (meter)'),
        DeclareLaunchArgument('auto_boundary', default_value='true', description='Otomatis trigger batas padang rumput'),
        DeclareLaunchArgument('auto_boundary_delay', default_value='7.5', description='Detik tunda sebelum kirim auto-boundary'),
        DeclareLaunchArgument('boundary_x_min', default_value='-7.0', description='Batas area meadow X min'),
        DeclareLaunchArgument('boundary_x_max', default_value='7.0', description='Batas area meadow X max'),
        DeclareLaunchArgument('boundary_y_min', default_value='-5.0', description='Batas area meadow Y min'),
        DeclareLaunchArgument('boundary_y_max', default_value='5.0', description='Batas area meadow Y max'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        OpaqueFunction(function=launch_setup),
    ])
