import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_mode = LaunchConfiguration('lidar_mode')

    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    # Evaluasi xacro dengan substitusi dinamis di runtime
    robot_description_content = Command([
        'xacro ', xacro_file, ' ',
        'lidar_mode:=', lidar_mode
    ])

    robot_description_config = ParameterValue(robot_description_content, value_type=str)

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Gunakan simulasi (Gazebo) clock jika true'
        ),
        DeclareLaunchArgument(
            'lidar_mode',
            default_value='2d',
            description='Mode LiDAR: "2d" atau "3d"'
        ),
        node_robot_state_publisher
    ])
