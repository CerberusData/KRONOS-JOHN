from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package = 'canlink',
        #     node_executable = 'canlink_cabin',
        #     output = 'screen'
        # ),
        Node(
            package = 'canlink',
            node_executable = 'canlink_chassis',
            output = 'screen',
            arguments = [('__log_level:=info')]
        ),
        # Node(
        #     package = 'can_test',
        #     node_executable = 'test_node',
        #     output = 'screen'
        # ),
        Node(
            package = 'bosch_imu',
            node_executable = 'imu',
            output = 'screen',
            arguments = [('__log_level:=info')]
        ),
        Node(
            package = 'wheel_odometry',
            node_executable = 'wheel_odometry',
            output = 'screen',
            arguments = [('__log_level:=info')]
        ),
        Node(
            package = 'motion_control',
            node_executable = 'speed_controller',
            output = 'screen',
            arguments = [('__log_level:=info')]
        ),
        Node(
            package = 'supervisor',
            node_executable = 'supervisor',
            output = 'screen',
            arguments = [('__log_level:=info')]
        )
])