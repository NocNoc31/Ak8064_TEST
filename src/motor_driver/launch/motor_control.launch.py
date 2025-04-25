from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = "/home/ubuntu/AK80_64/src/motor_driver/config/setup.yaml"
    return LaunchDescription([
        Node(
            package='motor_driver',
            executable='motor_control_node',
            name='motor_control_node',
            parameters=[config]
        )
    ])