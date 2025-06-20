from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_controller',
            executable='lidar_processor',
            output='screen'
        ),
        Node(
            package='rover_controller',
            executable='proximity_warning',
            output='screen'
        ),
        Node(
            package='rover_controller',
            executable='emergency_stop',
            output='screen'
        )
    ])
