from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='ec545_final',
            executable='spawner',
            name='spawner'
        ),
        # Node(
        #     package='ec545_final',
        #     executable='odometryPublisher',
        #     name='odometryPublisher'
        # ),
        Node(
            package='ec545_final',
            executable='planner',
            name='planner'
        ),
    ])