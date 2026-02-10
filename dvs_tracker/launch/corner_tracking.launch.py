from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dvs_tracker',
            executable='arc_star_detector',
            name='arc_star_detector',
            output='screen',
            parameters=[{
                'decay_time': 0.02,
                'filter_threshold': 0.01,
            }],
        ),
        Node(
            package='dvs_tracker',
            executable='corner_tracker',
            name='corner_tracker',
            output='screen',
            parameters=[{
                'track_window': 1.0,
            }],
        ),
    ])
