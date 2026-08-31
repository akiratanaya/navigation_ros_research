import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, EqualsSubstitution
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
    rviz_arg = DeclareLaunchArgument(
        'rviz_sim',
        default_value='true',
        description='Buka RViz2 secara otomatis jika true'
    )

    world_path = PathJoinSubstitution([pkg_path, 'worlds', LaunchConfiguration('world')])
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'sim.rviz')

    # 1. Gazebo Sim (Harmonic) - start pertama
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    # 2. Bridge Dasar (selalu aktif: Clock, CmdVel, Odom, JointStates, TF, IMU, Camera)
    base_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    # Bridge Mode 2D: Hanya /scan (sensor_msgs/LaserScan)
    bridge_2d = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('lidar_mode'), '2d')),
        output='screen'
    )

    # Bridge Mode 3D: /points (PointCloud2 16-channel)
    bridge_3d = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/points/points', '/points'),
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('lidar_mode'), '3d')),
        output='screen'
    )

    # Konversi 3D PointCloud2 ke 2D LaserScan /scan untuk SLAM dan Navigasi
    p2l_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/points'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.2,
            'min_height': 0.12,  # Memfilter lantai agar tidak menggambar lingkaran di tanah
            'max_height': 1.50,  # Mengambil dinding dan rintangan setinggi 1.5m
            'angle_min': -3.141592,
            'angle_max': 3.141592,
            'angle_increment': 0.008726,  # 0.5° per ray
            'scan_time': 0.1,
            'range_min': 0.30,   # Melewati radius bodi robot (0.18m) & ground hit dekat
            'range_max': 50.0,
            'use_inf': True,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('lidar_mode'), '3d')),
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
        period=5.5,
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

    # 5. RViz2 Node — otomatis terbuka bersama simulasi
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                condition=IfCondition(LaunchConfiguration('rviz_sim')),
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
        rviz_arg,
        gz_sim,            # 1. Gazebo start
        base_bridge_node,  # 2. Bridge dasar (Clock, Odom, CmdVel, TF, Camera, IMU)
        bridge_2d,         # 3a. Bridge 2D: /scan (hanya aktif jika lidar_mode:=2d)
        bridge_3d,         # 3b. Bridge 3D: /points (hanya aktif jika lidar_mode:=3d)
        p2l_node,          # 3c. Konversi /points -> /scan (hanya aktif jika lidar_mode:=3d)
        rsp,               # 4. RSP setelah 2 detik
        spawn_entity,      # 5. Spawn robot setelah 3 detik
        rviz_node          # 6. RViz2 setelah 3.5 detik
    ])
