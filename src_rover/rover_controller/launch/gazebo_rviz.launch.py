from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    """Launch Gazebo, Robot, and related nodes."""    

    # Define directory paths
    package_name = 'rover_controller'
    robot_file = os.path.join(get_package_share_directory(package_name), 'model', 'robot.xacro')
    world_file = os.path.join(get_package_share_directory(package_name), 'model','empty_world.world')
    rviz_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'rover_task_lidar_camera.rviz')

    # Process the XACRO into URDF
    robot_description = xacro.process_file(robot_file).toxml()

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={"world": world_file}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ])
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Spawn entity in Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover',
                    '-topic', 'robot_description',
                    '-x', '0',
                    '-y', '0',
                    '-z', '0.01'],
        output='screen'
    )
  
    # RViz2 with custom config
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        gazebo_client,
        joint_state_publisher,
        robot_state_publisher,
        spawn,
        rviz
    ])
