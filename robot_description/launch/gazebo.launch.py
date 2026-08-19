import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'turtlebot_custom.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'arena.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot_description.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    # Process xacro
    robot_description_content = Command(['xacro ', xacro_file])

    # Set GZ_SIM_RESOURCE_PATH to find models
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'worlds')]
    )

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_x = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z = DeclareLaunchArgument('z_pose', default_value='0.0')

    # Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': True,
        }],
    )

    # Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot_custom',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen',
    )

    #  ROS-Gazebo Bridge 
    # Bridge Gazebo topics to ROS2 topics
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Differential drive (cmd_vel → Gazebo, odom ← Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # LiDAR scan
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # TF from Gazebo
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_x,
        declare_y,
        declare_z,
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
    ])
