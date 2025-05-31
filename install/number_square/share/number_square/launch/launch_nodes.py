from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='number_square',
            executable='pub_node',
            name='number_publisher'
        ),
        Node(
            package='number_square',
            executable='sub_node',
            name='number_subscriber'
        )
    ])