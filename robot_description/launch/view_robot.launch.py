import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description')

    # Include Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'robot_state_publisher.launch.py')]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Joint State Publisher GUI (untuk memutar 4 roda omni)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Path ke file konfigurasi rviz jika ada
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')
    rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen'
    )

    return LaunchDescription([
        rsp,
        joint_state_publisher_gui,
        rviz2
    ])
