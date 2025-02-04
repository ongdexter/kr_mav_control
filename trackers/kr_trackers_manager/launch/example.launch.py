"""
This launch file is just an example of how to launch the TrackersManager along with its lifecycle manager
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    robot_arg = DeclareLaunchArgument(
      'robot', default_value='quadrotor'
    )
    # Path to the configuration file
    config_file = FindPackageShare('kr_trackers_manager').find('kr_trackers_manager') + '/config/trackers_manager.yaml'

    container = ComposableNodeContainer(
        name="trackers_manager_container",
        namespace=LaunchConfiguration('robot'),
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions= [
            ComposableNode(
                package="kr_trackers_manager",
                plugin="TrackersManager",
                name="trackers_manager",
                parameters=[config_file],
                namespace=LaunchConfiguration('robot')
            ),
            ComposableNode(
                package="kr_trackers_manager",
                plugin="TrackersManagerLifecycleManager",
                name="lifecycle_manager",
                namespace=LaunchConfiguration('robot'),
                parameters=[
                    {'node_name': "trackers_manager"}
                ]
            )
        ],
        output='screen'
    )
    

    return LaunchDescription([
        robot_arg,
        container,
    ])
