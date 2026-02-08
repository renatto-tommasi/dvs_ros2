from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dvs_view_ros2',
            executable='histogram_2D',
            name='histogram_2D',
            output='screen',
        ),
        Node(
            package='dvs_tracker',
            executable='efast_detector',
            name='efast_detector',
            output='screen',
            parameters=[{
                'fast_threshold': 50,
                'non_max_suppression': True,
            }],
        ),
    ])
