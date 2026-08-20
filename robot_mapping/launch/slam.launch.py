import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    pkg_mapping_path = get_package_share_directory('robot_mapping')
    slam_params_file = os.path.join(pkg_mapping_path, 'config', 'mapper_params_online_async.yaml')

    # Gunakan official online_async_launch.py dari slam_toolbox
    # yang menangani Lifecycle transition (configure & activate) secara otomatis
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gunakan simulasi/Gazebo clock jika bernilai true'
        ),
        slam_launch
    ])