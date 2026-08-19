import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
    
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
       
    # Ambil path konfigurasi dari package robot_mapping
    pkg_mapping_path = get_package_share_directory('robot_mapping')
    slam_params_file = os.path.join(pkg_mapping_path, 'config','mapper_params_online_async.yaml')

    # Node SLAM Toolbox
    start_async_slam_toolbox_node = Node(
    parameters=[
        slam_params_file,
        {'use_sim_time': use_sim_time}
    ],
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
           'use_sim_time',
            default_value='true',
            description='Gunakan simulasi/Gazebo clock jika bernilai true'
        ),
        start_async_slam_toolbox_node
    ])