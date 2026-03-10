import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = os.path.expanduser('~/ros_hackathon/drone_urdf/drone.urdf')
    world_file = os.path.expanduser('~/ros_hackathon/worlds/drone_world.sdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([

        # Launch Gazebo with our world
        ExecuteProcess(
            cmd=['gz', 'sim', world_file],
            output='screen'
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Spawn drone into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'quadrotor',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ],
            output='screen'
        ),

    ])
