import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('robot_description')
    pkg_mapping     = get_package_share_directory('robot_mapping')
    pkg_coverage    = get_package_share_directory('robot_coverage')
    pkg_navigation  = get_package_share_directory('robot_navigation')

    default_map = os.path.join(pkg_mapping, 'map', 'map_turtlehouse.yaml')
    default_rviz = os.path.join(pkg_coverage, 'rviz', 'coverage.rviz')

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_house.world',
        description='Nama file world Gazebo'
    )
    lidar_mode_arg = DeclareLaunchArgument(
        'lidar_mode',
        default_value='2d',
        description='Mode LiDAR: "2d" atau "3d"'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gunakan simulasi clock jika true'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path ke file map YAML'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Buka RViz2 otomatis jika true'
    )
    decomp_method_arg = DeclareLaunchArgument(
        'decomp_method',
        default_value='boustrophedon',
        description='Metode dekomposisi: "boustrophedon", "trapezoidal", atau "none"'
    )
    route_pattern_arg = DeclareLaunchArgument(
        'route_pattern',
        default_value='or_tools',
        description='Pola rute: "or_tools", "boustrophedon", "snake", atau "spiral"'
    )

    use_sim_time  = LaunchConfiguration('use_sim_time')
    lidar_mode    = LaunchConfiguration('lidar_mode')
    world         = LaunchConfiguration('world')
    map_file      = LaunchConfiguration('map')
    decomp_method = LaunchConfiguration('decomp_method')
    route_pattern = LaunchConfiguration('route_pattern')

    # 1. Gazebo Simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'sim.launch.py')]),
        launch_arguments={
            'world': world,
            'lidar_mode': lidar_mode,
            'use_sim_time': use_sim_time,
            'rviz_sim': 'false',
        }.items()
    )

    # 2. Coverage Pipeline (Map Server + Lifecycle Manager + Collector + Coverage Server)
    coverage_pipeline_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_coverage, 'launch', 'coverage_pipeline.launch.py')]),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'use_rviz': 'false',  # RViz dihandle terpisah dengan delay
                    'auto_compute': 'true',
                    'decomp_method': decomp_method,
                    'route_pattern': route_pattern,
                }.items()
            )
        ]
    )

    # 3. RViz2 dengan coverage configuration
    rviz_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_coverage',
                arguments=['-d', default_rviz],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('rviz')),
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        lidar_mode_arg,
        use_sim_time_arg,
        map_arg,
        rviz_arg,
        decomp_method_arg,
        route_pattern_arg,
        sim_launch,
        coverage_pipeline_launch,
        rviz_node,
    ])
