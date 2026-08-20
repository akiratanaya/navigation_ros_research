from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    resolution = LaunchConfiguration('resolution')
    frame_id = LaunchConfiguration('frame_id')

    # Node OctoMap 3D Occupancy Voxel Mapping
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        remappings=[
            ('cloud_in', '/points'),
        ],
        parameters=[{
            'resolution': resolution,
            'frame_id': frame_id,
            'sensor_model/max_range': 25.0,
            'sensor_model/hit': 0.7,
            'sensor_model/miss': 0.4,
            'sensor_model/min': 0.12,
            'sensor_model/max': 0.97,
            # Filter lantai/tanah otomatis
            'filter_ground': True,
            'ground_filter/distance': 0.05,
            'ground_filter/angle': 0.15,
            'ground_filter/plane_distance': 0.08,
            # Batas ketinggian Z (m)
            'pointcloud_min_z': -0.1,
            'pointcloud_max_z': 2.5,
            'occupancy_min_z': 0.05,
            'occupancy_max_z': 2.2,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gunakan simulasi clock jika true'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolusi voxel 3D dalam meter (default: 0.05m = 5cm)'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='odom',
            description='Frame acuan peta 3D (default: odom)'
        ),
        octomap_server_node
    ])
