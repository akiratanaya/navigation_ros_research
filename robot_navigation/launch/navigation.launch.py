import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_navigation = get_package_share_directory('robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    # Nav2 stack — tanpa AMCL (localization disuplai dari SLAM Toolbox)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
            'use_composition': 'False',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gunakan simulasi clock jika true'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'),
            description='Full path ke file parameter Nav2'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path ke file map YAML (opsional; kosongkan jika live SLAM)'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Otomatis aktivasi lifecycle nodes jika true'
        ),
        nav2_launch,
    ])
