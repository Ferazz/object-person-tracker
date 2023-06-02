from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_paintball_person_tracker',
            executable='camera',
        ),
        Node(
            package='ros_paintball_person_tracker',
            executable='yolo',
        ),
        Node(
            package='ros_paintball_person_tracker',
            executable='visualizer',
        )
    ])