from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'canlink',
            node_executable = 'canlink_cabin',
            output = 'screen'
        ),
        Node(
            package = 'canlink',
            node_executable = 'canlink_chassis',
            output = 'screen'
        ),
        Node(
            package = 'can_test',
            node_executable = 'test_node',
            output = 'screen'
        )
    ])