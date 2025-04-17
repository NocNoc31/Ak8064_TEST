import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="motor_driver",
            executable="motor_test_node",
            name="motor_test_node",
            output="screen",
        ),
    ])
