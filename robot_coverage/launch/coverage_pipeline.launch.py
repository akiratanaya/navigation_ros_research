#!/usr/bin/env python3
"""
Launch file untuk pipeline coverage path planning (Fields2Cover):
  - nav2_map_server (load map YAML & PGM)
  - nav2_lifecycle_manager (otomatis activate map_server tanpa step manual)
  - field_boundary_collector (kumpulkan titik klik "Publish Point" RViz menjadi polygon)
  - coverage_server (otomatis compute coverage path saat boundary selesai)
  - rviz2 (terbuka otomatis dengan layout dan display lengkap untuk coverage)

Cara pakai (di dalam terminal zsh):
    ros2 launch robot_coverage coverage_pipeline.launch.py

Ubah decomposition & route pattern:
    ros2 launch robot_coverage coverage_pipeline.launch.py decomp_method:=trapezoidal route_pattern:=snake
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_coverage = get_package_share_directory('robot_coverage')
    pkg_mapping = get_package_share_directory('robot_mapping')

    default_map_path = os.path.join(pkg_mapping, 'map', 'map_turtlehouse.yaml')
    default_rviz_config = os.path.join(pkg_coverage, 'rviz', 'coverage.rviz')

    # ── Arguments ─────────────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Path lengkap ke file .yaml map yang mau di-load'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Gunakan simulasi clock jika true'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Otomatis buka RViz2 dengan konfigurasi coverage'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path file konfigurasi RViz (.rviz)'
    )
    auto_compute_arg = DeclareLaunchArgument(
        'auto_compute',
        default_value='true',
        description='Otomatis generate coverage path saat /finish_field_boundary dipanggil'
    )
    robot_width_arg = DeclareLaunchArgument(
        'robot_width',
        default_value='0.1',
        description='Lebar bodi robot (meter)'
    )
    cov_width_arg = DeclareLaunchArgument(
        'cov_width',
        default_value='0.3',
        description='Lebar jangkauan swath / coverage tool (meter)'
    )
    headland_swaths_arg = DeclareLaunchArgument(
        'headland_swaths',
        default_value='1',
        description='Jumlah putaran headland di pinggir lahan'
    )
    route_pattern_arg = DeclareLaunchArgument(
        'route_pattern',
        default_value='or_tools',
        description='Pola rute: "or_tools", "boustrophedon", "snake", atau "spiral"'
    )
    decomp_method_arg = DeclareLaunchArgument(
        'decomp_method',
        default_value='boustrophedon',
        description='Metode dekomposisi lahan: "boustrophedon", "trapezoidal", atau "none"'
    )
    split_angle_arg = DeclareLaunchArgument(
        'split_angle',
        default_value='1.5707963',
        description='Sudut pemotong dekomposisi dalam radian (default: 1.5707963 ~ 90 deg)'
    )
    spiral_width_arg = DeclareLaunchArgument(
        'spiral_width',
        default_value='4',
        description='Lebar lompatan jalur pola spiral (jumlah jalur per blok spiral)'
    )

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    auto_compute = LaunchConfiguration('auto_compute')
    robot_width = LaunchConfiguration('robot_width')
    cov_width = LaunchConfiguration('cov_width')
    headland_swaths = LaunchConfiguration('headland_swaths')
    route_pattern = LaunchConfiguration('route_pattern')
    decomp_method = LaunchConfiguration('decomp_method')
    split_angle = LaunchConfiguration('split_angle')
    spiral_width = LaunchConfiguration('spiral_width')

    # ── 1. Map Server Node ───────────────────────────────────────────────────
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── 2. Lifecycle Manager untuk Map Server (Standar Nav2) ──────────────────
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_coverage',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }],
    )

    # ── 3. Field Boundary Collector Node ─────────────────────────────────────
    field_boundary_collector_node = Node(
        package='robot_coverage',
        executable='field_boundary_collector',
        name='field_boundary_collector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 4. Coverage Server Node (Fields2Cover Wrapper) ───────────────────────
    coverage_server_node = Node(
        package='robot_coverage',
        executable='coverage_server',
        name='coverage_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_compute': auto_compute,
            'robot_width': robot_width,
            'cov_width': cov_width,
            'headland_swaths': headland_swaths,
            'route_pattern': route_pattern,
            'decomp_method': decomp_method,
            'split_angle': split_angle,
            'spiral_width': spiral_width,
        }],
    )

    # ── 5. RViz2 dengan Config Coverage (Delay 1.0s agar map_server aktif) ───
    rviz_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_coverage',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_rviz),
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        use_rviz_arg,
        rviz_config_arg,
        auto_compute_arg,
        robot_width_arg,
        cov_width_arg,
        headland_swaths_arg,
        route_pattern_arg,
        decomp_method_arg,
        split_angle_arg,
        spiral_width_arg,
        map_server_node,
        lifecycle_manager_node,
        field_boundary_collector_node,
        coverage_server_node,
        rviz_node,
    ])