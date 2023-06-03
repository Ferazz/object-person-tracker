from launch import LaunchDescription
from launch_ros.actions import Node

# Shared parameters between Nodes
# FIXME: Better way to do this?
CENTER_OFFSET = (0, 0)
RADIUS = 30

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
            parameters=[
                {'radius': RADIUS},
                {'center_offset': CENTER_OFFSET}
            ],
        ),
        Node(
            package='ros_paintball_person_tracker',
            executable='robot_controller',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'radius': RADIUS},
                {'center_offset': CENTER_OFFSET}
            ],
        ),
        Node(
            package='ros_paintball_person_tracker',
            executable='serial',
        ),
    ])