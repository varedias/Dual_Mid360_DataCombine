from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='merge_cloud',
            executable='merge_cloud_node',
            name='merge_cloud_node',
            output='screen',
            parameters=[],
            remappings=[],
        ),
    ])

