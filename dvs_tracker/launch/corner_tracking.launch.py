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
                'decay_time': 0.01,
                'filter_threshold': 0.05,
                'nms_threshold': 0.05,
            }],
        ),
        Node(
            package='dvs_tracker',
            executable='corner_tracker',
            name='corner_tracker',
            output='screen',
            parameters=[{
                'track_window': 0.1,
                'd_conn': 8.0,
                'rho_thresh': 15,
                'max_track_length': 10,
            }],
        ),
    ])
