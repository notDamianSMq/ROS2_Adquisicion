from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='project_pkg',
            executable='sensor_processor',
            name='sensor_processor_node'
        ),
        Node(
            package='project_pkg',
            executable='obstacle_detector',
            name='obstacle_detector_node'
        ),
    ])