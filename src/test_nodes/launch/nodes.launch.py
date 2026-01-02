from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node A 실행
        Node(
            package='test_nodes',
            executable='node_a',
            name='node_a',
            output='screen',
        ),
        
        # Node B 실행
        Node(
            package='test_nodes',
            executable='node_b',
            name='node_b',
            output='screen',
        ),
    ])