import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='mmg_module',
            executable='controller_node',
            name='contorller_node'
        ),
    ])