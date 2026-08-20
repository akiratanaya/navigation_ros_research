from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Menjalankan teleop_twist_keyboard di jendela terminal xterm pop-up
    teleop_process = ExecuteProcess(
        cmd=[
            'xterm',
            '-title', 'Teleop Keyboard - AutoNav Bot',
            '-geometry', '70x20',
            '-fa', 'Monospace',
            '-fs', '11',
            '-bg', '#1e1e2e',
            '-fg', '#cdd6f4',
            '-e', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'
        ],
        output='screen'
    )

    return LaunchDescription([
        teleop_process
    ])
