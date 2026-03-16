import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros_hack')
    world_file = os.path.join(pkg_share, 'worlds', 'aerial_nav.world')

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
            additional_env={'GZ_SIM_RESOURCE_PATH': pkg_share},
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/world/aerial_nav_world/model/X4/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/model/X4/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '/model/X4/gazebo/command/twist@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/X4/enable@std_msgs/msg/Bool]gz.msgs.Boolean',
            ],
            output='screen',
        ),

        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='image_bridge',
            arguments=['/world/aerial_nav_world/model/X4/model/X4/link/base_link/sensor/camera_front/image'],
            output='screen',
        ),

    ])
