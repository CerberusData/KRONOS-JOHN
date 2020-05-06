from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bosch_imu',
            node_executable='imu_node',
            node_name='imu',
            output='screen')
    ])