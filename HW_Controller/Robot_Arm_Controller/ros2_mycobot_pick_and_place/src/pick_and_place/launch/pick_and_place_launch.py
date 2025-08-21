from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pick_and_place',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='pick_and_place',
            executable='detection_node',
            name='detection_node',
            output='screen'
        ),
        Node(
            package='pick_and_place',
            executable='transform_node',
            name='transform_node',
            output='screen'
        ),
        Node(
            package='pick_and_place',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
    ])
