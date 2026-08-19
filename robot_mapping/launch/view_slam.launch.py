import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_mapping_path = get_package_share_directory('robot_mapping')

    # 1. Jalankan SLAM Toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_mapping_path, 'launch', 'slam.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Jalankan RViz2 dengan use_sim_time
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gunakan simulation clock jika bernilai true'
        ),
        slam,
        rviz2
    ])
