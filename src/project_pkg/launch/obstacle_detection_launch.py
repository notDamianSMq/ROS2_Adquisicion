from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project_pkg',
            executable='listener_points',
            name='cloud_points_node',
            emulate_tty=True,
        ),
        Node(
            package='project_pkg',
            executable='fusion_collector',
            name='fusion_node',
            emulate_tty=True,
        ),
    ])