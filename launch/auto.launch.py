from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ── 3. Autonomous drone node ───────────────────────────────────
        Node(
            package='ros_hack',
            executable='autonomous_x4',
            name='autonomous_x4',
            output='screen',
        ),
    ])
