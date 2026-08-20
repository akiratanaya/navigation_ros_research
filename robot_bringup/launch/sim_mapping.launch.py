import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('robot_description')
    pkg_mapping = get_package_share_directory('robot_mapping')
    pkg_bringup = get_package_share_directory('robot_bringup')

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_house.world',
        description='Nama file world Gazebo'
    )
    lidar_mode_arg = DeclareLaunchArgument(
        'lidar_mode',
        default_value='2d',
        description='Mode LiDAR: "2d" (LaserScan) atau "3d" (PointCloud2 VLP-16)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gunakan simulasi clock jika true'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Buka RViz2 otomatis jika true'
    )
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='true',
        description='Buka terminal pop-up Teleop Keyboard otomatis jika true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_mode = LaunchConfiguration('lidar_mode')
    world = LaunchConfiguration('world')

    # 1. Jalankan Simulasi Gazebo (tanpa RViz internal agar tidak ganda)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'sim.launch.py')]),
        launch_arguments={
            'world': world,
            'lidar_mode': lidar_mode,
            'use_sim_time': use_sim_time,
            'rviz_sim': 'false' # Matikan RViz sim agar RViz SLAM yang aktif
        }.items()
    )

    # 2. Jalankan SLAM Toolbox (delay 2.5 detik agar Gazebo & Clock sudah siap)
    slam_launch = TimerAction(
        period=2.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_mapping, 'launch', 'slam.launch.py')]),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # 3. Jalankan OctoMap 3D Mapping (hanya aktif pada mode 3D LiDAR, delay 2.5 detik)
    octomap_launch = TimerAction(
        period=2.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_mapping, 'launch', 'octomap_mapping.launch.py')]),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=IfCondition(EqualsSubstitution(LaunchConfiguration('lidar_mode'), '3d'))
            )
        ]
    )

    # 4. Jalankan RViz2 dengan konfigurasi SLAM Mapping (langsung buka di awal)
    rviz_config_file = os.path.join(pkg_mapping, 'rviz', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # 5. Jalankan Teleop Keyboard di terminal pop-up (delay 3.0 detik)
    teleop_action = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'xterm',
                    '-title', 'AutoNav Bot - Teleop Keyboard Controller',
                    '-geometry', '75x22',
                    '-fa', 'Monospace',
                    '-fs', '11',
                    '-bg', '#1e1e2e',
                    '-fg', '#cdd6f4',
                    '-e', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'
                ],
                condition=IfCondition(LaunchConfiguration('teleop')),
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        lidar_mode_arg,
        use_sim_time_arg,
        rviz_arg,
        teleop_arg,
        sim_launch,
        slam_launch,
        octomap_launch,
        rviz_node,
        teleop_action
    ])
