from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Static TF: map -> dvs_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'dvs_frame'],
        ),
        # Voxel grid node
        Node(
            package='dvs_view_ros2',
            executable='voxel_grid',
        ),
    ])
