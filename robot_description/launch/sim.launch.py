import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot_description'
    pkg_path = get_package_share_directory(pkg_name)
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Setup resource path agar Gazebo Harmonic dapat menemukan model dan world
    models_path = os.path.join(pkg_path, 'models')
    worlds_path = os.path.join(pkg_path, 'worlds')
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = f"{models_path}:{worlds_path}:{pkg_path}:{existing_resource_path}"

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_house.world',
        description='Nama file world (default: turtlebot3_house.world)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gunakan simulasi clock jika true'
    )
    x_arg = DeclareLaunchArgument('x', default_value='-2.0', description='Posisi spawn robot X')
    y_arg = DeclareLaunchArgument('y', default_value='1.0', description='Posisi spawn robot Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.05', description='Posisi spawn robot Z')
    lidar_mode_arg = DeclareLaunchArgument(
        'lidar_mode',
        default_value='2d',
        description='Mode LiDAR: "2d" (LaserScan flat) atau "3d" (PointCloud2 VLP-16)'
    )

    world_path = PathJoinSubstitution([pkg_path, 'worlds', LaunchConfiguration('world')])

    # 1. Gazebo Sim (Harmonic) - start pertama
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    # 2. Bridge antara ROS 2 Jazzy dan Gazebo Sim (Harmonic) - start segera setelah Gazebo
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock — paling penting, harus tersedia sebelum RSP
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Velocity command
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Joint States
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # TF (odom -> base_footprint dari DiffDrive Gazebo)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # ── 2D & 3D LiDAR (LaserScan) ──────────────────────────────────
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # ── 3D LiDAR (PointCloud2 16-channel) ─────────────────────────
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/pointcloud/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            # ── IMU Data ───────────────────────────────────────────────────
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # ── RGBD Camera ────────────────────────────────────────────────
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # 3. Robot State Publisher — delay 2 detik agar /clock dari Gazebo sudah tersedia
    rsp = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'robot_state_publisher.launch.py')]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'lidar_mode': LaunchConfiguration('lidar_mode')
                }.items()
            )
        ]
    )

    # 4. Spawn Robot — langsung kirim string xacro agar tidak bergantung pada topik
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description_content = Command([
        'xacro ', xacro_file, ' ',
        'lidar_mode:=', LaunchConfiguration('lidar_mode')
    ])

    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-string', robot_description_content,
                    '-name', 'autonav_bot',
                    '-x', LaunchConfiguration('x'),
                    '-y', LaunchConfiguration('y'),
                    '-z', LaunchConfiguration('z')
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        set_gz_resource_path,
        world_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        lidar_mode_arg,
        gz_sim,       # 1. Gazebo start
        bridge_node,  # 2. Bridge (termasuk /clock)
        rsp,          # 3. RSP setelah 3 detik (sim clock sudah ada)
        spawn_entity  # 4. Spawn robot setelah 5 detik (world sudah loaded)
    ])
