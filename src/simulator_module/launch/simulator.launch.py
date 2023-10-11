from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulator_module',
            executable='actuator_node',
            name='actuator_node',
            output='screen',
        ),
        Node(
            package='simulator_module',
            executable='controller_node',
            name='controller_node',
            output='screen',
        ),
        Node(
            package='simulator_module',
            executable='model_node',
            name='model_node',
            output='screen',
        ),
        Node(
            package='simulator_module',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
        ),
        ])