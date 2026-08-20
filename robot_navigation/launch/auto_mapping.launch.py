import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('robot_navigation')
    pkg_mapping    = get_package_share_directory('robot_mapping')

    use_sim_time     = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    autostart        = LaunchConfiguration('autostart')

    # 1. Nav2 stack (controller, planner, bt_navigator, dsb.) — delay 2s
    nav2_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')]
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'nav2_params_file': nav2_params_file,
                    'autostart': autostart,
                }.items()
            )
        ]
    )

    # 2. Frontier Auto-Explorer node — delay 5s (Nav2 dan peta harus sudah siap)
    explorer_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='robot_navigation',
                executable='auto_explorer.py',
                name='frontier_explorer',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_base_frame': 'base_footprint',
                    'map_frame': 'map',
                    'min_frontier_size': 10,
                    'exploration_timeout': 30.0,
                    'goal_tolerance': 0.3,
                }],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gunakan simulasi clock'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'),
            description='File parameter Nav2'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Otomatis aktivasi lifecycle nodes'
        ),
        nav2_launch,
        explorer_node,
    ])
