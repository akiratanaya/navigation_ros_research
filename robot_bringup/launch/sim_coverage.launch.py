import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description  = get_package_share_directory('robot_description')
    pkg_mapping      = get_package_share_directory('robot_mapping')
    pkg_coverage     = get_package_share_directory('robot_coverage')
    pkg_navigation   = get_package_share_directory('robot_navigation')

    default_map = os.path.join(pkg_mapping, 'map', 'map_turtlehouse.yaml')
    default_rviz = os.path.join(pkg_coverage, 'rviz', 'coverage.rviz')
    default_rviz_cmu = os.path.join(pkg_coverage, 'rviz', 'coverage_cmu.rviz')
    nav2_params_file = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')

    # ── Arguments ─────────────────────────────────────────────────────────────
    nav_backend_arg = DeclareLaunchArgument(
        'nav_backend',
        default_value='nav2',
        description='Backend navigasi: "nav2" (default) atau "cmu"'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_house.world',
        description='Nama file world Gazebo'
    )
    lidar_mode_arg = DeclareLaunchArgument(
        'lidar_mode',
        default_value='auto',
        description='Mode LiDAR: "auto" (default: "3d" untuk cmu, "2d" untuk nav2), atau "2d"/"3d"'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Gunakan simulasi clock jika true'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path ke file map YAML'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Buka RViz2 otomatis jika true'
    )
    decomp_method_arg = DeclareLaunchArgument(
        'decomp_method',
        default_value='boustrophedon',
        description='Metode dekomposisi: "boustrophedon", "trapezoidal", atau "none"'
    )
    route_pattern_arg = DeclareLaunchArgument(
        'route_pattern',
        default_value='boustrophedon',
        description='Pola rute: "boustrophedon", "snake", "spiral", atau "or_tools"'
    )
    auto_navigate_arg = DeclareLaunchArgument(
        'auto_navigate',
        default_value='true',
        description='Otomatis jalankan navigasi Nav2 begitu coverage path digenerate'
    )
    x_arg = DeclareLaunchArgument('x', default_value='-2.0', description='Posisi spawn robot X')
    y_arg = DeclareLaunchArgument('y', default_value='1.0', description='Posisi spawn robot Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.05', description='Posisi spawn robot Z')

    use_sim_time  = LaunchConfiguration('use_sim_time')
    nav_backend   = LaunchConfiguration('nav_backend')
    lidar_mode    = LaunchConfiguration('lidar_mode')
    world         = LaunchConfiguration('world')
    map_file      = LaunchConfiguration('map')
    decomp_method = LaunchConfiguration('decomp_method')
    route_pattern = LaunchConfiguration('route_pattern')
    auto_navigate = LaunchConfiguration('auto_navigate')
    spawn_x       = LaunchConfiguration('x')
    spawn_y       = LaunchConfiguration('y')
    spawn_z       = LaunchConfiguration('z')

    is_nav2 = EqualsSubstitution(nav_backend, 'nav2')
    is_cmu  = EqualsSubstitution(nav_backend, 'cmu')

    effective_lidar_mode = PythonExpression([
        "'3d' if '", lidar_mode, "' == '3d' or ('", lidar_mode, "' == 'auto' and '", nav_backend, "' == 'cmu') else '2d'"
    ])

    show_rviz_nav2 = IfCondition(PythonExpression(["'", LaunchConfiguration('rviz'), "' == 'true' and '", nav_backend, "' == 'nav2'"]))
    show_rviz_cmu  = IfCondition(PythonExpression(["'", LaunchConfiguration('rviz'), "' == 'true' and '", nav_backend, "' == 'cmu'"]))

    # ── 1. Gazebo Simulation (Spawn Robot & Sensors) ──────────────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_description, 'launch', 'sim.launch.py')]),
        launch_arguments={
            'world': world,
            'lidar_mode': effective_lidar_mode,
            'use_sim_time': use_sim_time,
            'x': spawn_x,
            'y': spawn_y,
            'z': spawn_z,
            'rviz_sim': 'false',
        }.items()
    )

    # ── 2A. Nav2 Localization (AMCL + Map Server) (Delay 6.0s, nav_backend == 'nav2') ─
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    localization_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
                ]),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                    'autostart': 'true',
                    'use_composition': 'False',
                }.items(),
                condition=IfCondition(is_nav2)
            )
        ]
    )

    # ── 3A. Nav2 Navigation Stack (Delay 10.0s, nav_backend == 'nav2') ───────
    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'nav2_params_file': nav2_params_file,
                    'autostart': 'true',
                }.items(),
                condition=IfCondition(is_nav2)
            )
        ]
    )

    # ── 4A. Coverage Pipeline for Nav2 (Delay 13.0s, nav_backend == 'nav2') ───
    coverage_pipeline_launch = TimerAction(
        period=13.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_coverage, 'launch', 'coverage_pipeline.launch.py')]),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': use_sim_time,
                    'use_rviz': 'false',
                    'use_map_server': 'false',
                    'auto_compute': 'true',
                    'use_navigator': 'true',
                    'auto_navigate': auto_navigate,
                    'decomp_method': decomp_method,
                    'route_pattern': route_pattern,
                }.items(),
                condition=IfCondition(is_nav2)
            )
        ]
    )

    # ── 5A. RViz2 for Nav2 (Delay 14.0s) ─────────────────────────────────────
    rviz_nav2_node = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_coverage',
                arguments=['-d', default_rviz],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=show_rviz_nav2,
                output='screen'
            )
        ]
    )

    # ── 2B. CMU Autonomy Stack (Delay 6.0s, nav_backend == 'cmu') ─────────────
    cmu_autonomy_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_coverage, 'launch', 'cmu_autonomy.launch.py')]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'decomp_method': decomp_method,
                    'route_pattern': route_pattern,
                    'auto_navigate': auto_navigate,
                }.items(),
                condition=IfCondition(is_cmu)
            )
        ]
    )

    # ── 3B. RViz2 for CMU (Delay 8.0s, 3D Orbit View) ────────────────────────
    rviz_cmu_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_coverage_cmu',
                arguments=['-d', default_rviz_cmu],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=show_rviz_cmu,
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        nav_backend_arg,
        world_arg,
        lidar_mode_arg,
        use_sim_time_arg,
        map_arg,
        rviz_arg,
        decomp_method_arg,
        route_pattern_arg,
        auto_navigate_arg,
        x_arg,
        y_arg,
        z_arg,
        sim_launch,
        # Nav2 backend actions
        localization_launch,
        nav2_launch,
        coverage_pipeline_launch,
        rviz_nav2_node,
        # CMU backend actions
        cmu_autonomy_launch,
        rviz_cmu_node,
    ])
