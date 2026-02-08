from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dvs_tracker',
            executable='corner_detector',
            name='corner_detector',
            output='screen',
            parameters=[{
                'decay_time': 0.03,
                'filter_threshold': 0.05,
            }],
        ),
    ])
