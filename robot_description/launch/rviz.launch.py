import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'turtlebot_custom.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_description.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
        }],
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
