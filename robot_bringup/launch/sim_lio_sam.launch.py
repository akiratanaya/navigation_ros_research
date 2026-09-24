import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess, OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    pkg_description = get_package_share_directory('robot_description')
    pkg_lio_sam = get_package_share_directory('lio_sam')

    world_val = context.launch_configurations.get('world', 'outdoor_park.world')
    x_val = context.launch_configurations.get('x', None)
    y_val = context.launch_configurations.get('y', None)
    z_val = context.launch_configurations.get('z', None)
    yaw_val = context.launch_configurations.get('yaw', None)

    # Menentukan posisi default spawn berdasarkan world jika tidak ditentukan eksplisit
    if 'turtlebot' in world_val:
        default_x = '-4.7'
        default_y = '-4.7'
        default_z = '0.05'
        default_yaw = '0.7854'
    else:
        # outdoor_park.world atau lingkungan 3D outdoor
        default_x = '0.0'
        default_y = '0.0'
        default_z = '0.15'
        default_yaw = '0.0'

    spawn_x = x_val if x_val is not None else default_x
    spawn_y = y_val if y_val is not None else default_y
    spawn_z = z_val if z_val is not None else default_z
    spawn_yaw = yaw_val if yaw_val is not None else default_yaw

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_rviz = LaunchConfiguration('rviz')
    run_teleop = LaunchConfiguration('teleop')

    # 1. Jalankan Simulasi Gazebo dengan 3D LiDAR (Velodyne VLP-16) dan IMU
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'sim.launch.py')]),
        launch_arguments={
            'world': world_val,
            'x': spawn_x,
            'y': spawn_y,
            'z': spawn_z,
            'yaw': spawn_yaw,
            'lidar_mode': '3d',           # Paksa mode 3D untuk Velodyne VLP-16
            'bridge_tf': 'false',         # LIO-SAM yang mempublikasikan TF odom -> robot
            'use_sim_time': use_sim_time,
            'rviz_sim': 'false'           # Matikan RViz bawaan sim agar RViz LIO-SAM yang aktif
        }.items()
    )

    # 2. Jalankan LIO-SAM SLAM (delay 3.0 detik agar Gazebo, Clock, /points, dan /imu/data sudah siap)
    lio_sam_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_lio_sam, 'launch', 'run.launch.py')]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'rviz': run_rviz,
                    'publish_robot_state': 'false'  # Sudah dipublikasikan oleh sim.launch.py
                }.items()
            )
        ]
    )

    # 3. Jalankan Teleop Keyboard di terminal pop-up (delay 3.5 detik)
    teleop_action = TimerAction(
        period=3.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'xterm',
                    '-title', 'AutoNav Bot - Teleop Keyboard Controller (LIO-SAM)',
                    '-geometry', '75x22',
                    '-fa', 'Monospace',
                    '-fs', '11',
                    '-bg', '#1e1e2e',
                    '-fg', '#cdd6f4',
                    '-e', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'
                ],
                condition=IfCondition(run_teleop),
                output='screen'
            )
        ]
    )

    return [sim_launch, lio_sam_launch, teleop_action]


def generate_launch_description():
    # Arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='outdoor_park.world',
        description='Nama file world Gazebo (default: outdoor_park.world)'
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
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Posisi spawn robot X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Posisi spawn robot Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.15', description='Posisi spawn robot Z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Rotasi yaw robot saat spawn (rad)')

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        rviz_arg,
        teleop_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        OpaqueFunction(function=launch_setup)
    ])
