import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description  = get_package_share_directory('robot_description')
    pkg_bringup      = get_package_share_directory('robot_bringup')
    pkg_coverage     = get_package_share_directory('robot_coverage')
    pkg_local_planner = get_package_share_directory('local_planner')
    pkg_terrain_analysis = get_package_share_directory('terrain_analysis')
    pkg_terrain_analysis_ext = get_package_share_directory('terrain_analysis_ext')
    pkg_sensor_scan_gen = get_package_share_directory('sensor_scan_generation')
    pkg_tare_planner = get_package_share_directory('tare_planner')

    default_rviz = os.path.join(pkg_bringup, 'rviz', 'exploration.rviz')
    if not os.path.exists(default_rviz):
        default_rviz = os.path.join(pkg_tare_planner, 'rviz', 'tare_planner_ground.rviz')

    # ── Arguments ─────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='garage.world',
        description='Gazebo world file (e.g. garage.world, turtlebot3_house.world)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 automatically'
    )
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='indoor_small',
        description='TARE planner scenario (indoor_small, indoor_large, outdoor)'
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically start exploration on launch'
    )
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.6',
        description='Max robot speed (m/s)'
    )
    autonomy_speed_arg = DeclareLaunchArgument(
        'autonomy_speed',
        default_value='0.6',
        description='Autonomy navigation speed (m/s)'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.05', description='Robot spawn Z')

    world         = LaunchConfiguration('world')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    rviz          = LaunchConfiguration('rviz')
    scenario      = LaunchConfiguration('scenario')
    auto_start    = LaunchConfiguration('auto_start')
    max_speed     = LaunchConfiguration('max_speed')
    autonomy_speed= LaunchConfiguration('autonomy_speed')
    spawn_x       = LaunchConfiguration('x')
    spawn_y       = LaunchConfiguration('y')
    spawn_z       = LaunchConfiguration('z')

    # ── 1. Gazebo Simulation with 3D LiDAR ────────────────────────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'sim.launch.py')]),
        launch_arguments={
            'world': world,
            'lidar_mode': '3d',
            'use_sim_time': use_sim_time,
            'x': spawn_x,
            'y': spawn_y,
            'z': spawn_z,
            'rviz_sim': 'false',
        }.items()
    )

    # ── 2. CMU Base Autonomy Stack (Delay 6.0s) ──────────────────────────────
    cmu_sim_bridge_node = Node(
        package='robot_coverage',
        executable='cmu_sim_bridge',
        name='cmu_sim_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    terrain_analysis_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_terrain_analysis, 'launch', 'terrain_analysis.launch')
        )
    )

    terrain_analysis_ext_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_terrain_analysis_ext, 'launch', 'terrain_analysis_ext.launch')
        ),
        launch_arguments={'checkTerrainConn': 'true'}.items()
    )

    sensor_scan_gen_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_sensor_scan_gen, 'launch', 'sensor_scan_generation.launch')
        )
    )

    local_planner_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            pkg_local_planner, 'launch', 'local_planner.launch')
        ),
        launch_arguments={
            'config': 'standard',
            'realRobot': 'false',
            'autonomyMode': 'true',
            'twoWayDrive': 'false',
            'maxSpeed': max_speed,
            'autonomySpeed': autonomy_speed,
            'sensorOffsetX': '0.0',
            'sensorOffsetY': '0.0',
            'cameraOffsetZ': '0.0',
        }.items()
    )

    cmu_base_stack = TimerAction(
        period=6.0,
        actions=[
            cmu_sim_bridge_node,
            terrain_analysis_launch,
            terrain_analysis_ext_launch,
            sensor_scan_gen_launch,
            local_planner_launch,
        ]
    )

    # ── 3. RViz2 (Delay 8.0s) ────────────────────────────────────────────────
    rviz_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_exploration',
                arguments=['-d', default_rviz],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(rviz),
                output='screen'
            )
        ]
    )

    # ── 4. TARE Exploration Planner Node (Delay 10.0s) ───────────────────────
    def launch_tare_planner(context):
        scenario_name = str(scenario.perform(context))
        config_path = os.path.join(pkg_tare_planner, f'{scenario_name}.yaml')
        if not os.path.exists(config_path):
            config_path = os.path.join(pkg_tare_planner, 'config', f'{scenario_name}.yaml')

        tare_node = Node(
            package='tare_planner',
            executable='tare_planner_node',
            name='tare_planner_node',
            output='screen',
            parameters=[
                config_path,
                {
                    'use_sim_time': True,
                    'kAutoStart': True if auto_start.perform(context).lower() == 'true' else False,
                }
            ]
        )
        return [tare_node]

    tare_planner_action = TimerAction(
        period=10.0,
        actions=[
            OpaqueFunction(function=launch_tare_planner)
        ]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        rviz_arg,
        scenario_arg,
        auto_start_arg,
        max_speed_arg,
        autonomy_speed_arg,
        x_arg,
        y_arg,
        z_arg,
        sim_launch,
        cmu_base_stack,
        rviz_node,
        tare_planner_action,
    ])
