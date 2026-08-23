#!/usr/bin/env python3
"""
Launch file untuk pipeline coverage path planning secara end-to-end:
  - nav2_map_server (load map .pgm/.yaml) + auto lifecycle configure/activate
  - field_boundary_collector (kumpulkan titik klik "Publish Point" jadi polygon)
  - coverage_server (compute coverage path dari boundary)
  - (opsional) rviz2, dibuka polos tanpa config (setup Display manual sendiri)

Cara pakai:
    ros2 launch robot_coverage coverage_pipeline.launch.py

Override argument, contoh ganti map:
    ros2 launch robot_coverage coverage_pipeline.launch.py map:=/path/ke/map/lain.yaml

Tanpa RViz (misal RViz sudah dibuka manual):
    ros2 launch robot_coverage coverage_pipeline.launch.py use_rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
    EmitEvent,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    # Default map path - sesuaikan kalau lokasi map berbeda
    default_map_path = os.path.join(
        os.path.expanduser('~'),
        'robotics', 'delabo_itb', 'navigation_ros_ws',
        'src', 'navigation_ros_research', 'robot_mapping',
        'map', 'map_turtlehouse.yaml'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Path lengkap ke file .yaml map yang mau di-load'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Otomatis buka RViz2 atau tidak'
    )

    map_yaml_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')

    # 1. map_server (lifecycle node)
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}],
    )

    # Trigger configure begitu map_server start
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == map_server_node,
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    # Trigger activate begitu konfigurasi selesai (state jadi inactive)
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == map_server_node,
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # 2. field_boundary_collector
    field_boundary_collector_node = Node(
        package='robot_coverage',
        executable='field_boundary_collector',
        name='field_boundary_collector',
        output='screen',
    )

    # 3. coverage_server
    coverage_server_node = Node(
        package='robot_coverage',
        executable='coverage_server',
        name='coverage_server',
        output='screen',
    )

    # 4. RViz2 (opsional, delay sedikit supaya map_server & node lain sudah siap duluan)
    # Dibuka tanpa config custom - setup Display manual sendiri di RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    delayed_rviz = TimerAction(period=2.0, actions=[rviz_node])

    return LaunchDescription([
        use_rviz_arg,
        map_arg,
        map_server_node,
        configure_event,
        activate_event,
        field_boundary_collector_node,
        coverage_server_node,
        delayed_rviz,
    ])