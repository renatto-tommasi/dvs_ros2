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
                'radius': 5,
                'k_threshold': 0.01,
                'l_min': 4,
                'l_max': 8,
                'decay_time': 0.03,
            }],
        ),
    ])
