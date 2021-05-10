import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

this_pkg = 'rt2_assignment1';

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='rt2a1_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package = this_pkg,
                    plugin = this_pkg+'::RandPosServer',
                    name='position_server_node'),
                ComposableNode(
                    package = this_pkg,
                    plugin = this_pkg+'::StateMachine',
                    name='state_machine_node')
                
            ],
            output='screen',
        )
        
    return launch.LaunchDescription([container])

