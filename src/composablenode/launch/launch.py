import launch
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    two_nodes_container = launch_ros.actions.ComposableNodeContainer(
        name='two_nodes_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='composablenode',
                plugin='composable_node_tutorials::HeaderPublisherNode',
                name='header_publisher'
            ),
            launch_ros.descriptions.ComposableNode(
                package='composablenode',
                plugin='composable_node_tutorials::HeaderSubscriberNode',
                name='sub_same_component'
            )
        ],
        output='screen'
    )

    one_node_container = launch_ros.actions.ComposableNodeContainer(
        name='one_node_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='composablenode',
                plugin='composable_node_tutorials::HeaderSubscriberNode',
                name='sub_other_component'
            )
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        two_nodes_container,
        one_node_container
    ])