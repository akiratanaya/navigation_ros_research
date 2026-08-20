import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description  = get_package_share_directory('robot_description')
    pkg_mapping      = get_package_share_directory('robot_mapping')
    pkg_navigation   = get_package_share_directory('robot_navigation')
    pkg_bringup      = get_package_share_directory('robot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_mode   = LaunchConfiguration('lidar_mode')
    world        = LaunchConfiguration('world')

    # ── 1. Simulasi Gazebo ───────────────────────────────────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_description, 'launch', 'sim.launch.py')]
        ),
        launch_arguments={
            'world': world,
            'lidar_mode': lidar_mode,
            'use_sim_time': use_sim_time,
            'rviz_sim': 'false',
        }.items()
    )

    # ── 2. SLAM Toolbox (delay 2.5s) ────────────────────────────────────
    slam_launch = TimerAction(
        period=2.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_mapping, 'launch', 'slam.launch.py')]
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # ── 3. OctoMap (mode 3D saja, delay 2.5s) ───────────────────────────
    octomap_launch = TimerAction(
        period=2.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_mapping, 'launch', 'octomap_mapping.launch.py')]
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=IfCondition(EqualsSubstitution(lidar_mode, '3d'))
            )
        ]
    )

    # ── 4. Nav2 + Frontier Auto-Explorer (delay 8s) ──────────────────────
    # Nav2 butuh /map dari SLAM sebelum bisa configure global_costmap
    auto_mapping_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(pkg_navigation, 'launch', 'auto_mapping.launch.py')]
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # ── 5. RViz2 (config SLAM + Nav2 displays) ───────────────────────────
    rviz_config = os.path.join(pkg_mapping, 'rviz', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world',        default_value='turtlebot3_house.world'),
        DeclareLaunchArgument('lidar_mode',   default_value='2d',
                              description='Mode LiDAR: "2d" atau "3d"'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz',         default_value='true'),
        sim_launch,
        slam_launch,
        octomap_launch,
        auto_mapping_launch,
        rviz_node,
    ])
