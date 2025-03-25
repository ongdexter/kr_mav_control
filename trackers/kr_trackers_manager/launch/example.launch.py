"""
This launch file is just an example of how to launch the TrackersManager along with its lifecycle manager
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def print_config_file(context, *args, **kwargs):
    # Get the evaluated value of config_file
    config_file_value = LaunchConfiguration('config_file').perform(context)
    print(f"Config file path: {config_file_value}")  # Prints to the console
    return [LogInfo(msg=f"Config file path: {config_file_value}")]  # Logs via ROS

def generate_launch_description():

    robot_arg = DeclareLaunchArgument(
      'namespace', default_value=''
    )
    config_file_arg = DeclareLaunchArgument('config_file')
    # Path to the configuration file
    config_file = LaunchConfiguration('config_file')

    container = ComposableNodeContainer(
        name="trackers_manager_container",
        namespace=LaunchConfiguration('namespace'),
        package="rclcpp_components",
        executable="component_container_mt",
        #prefix=['xterm -e gdb --args'],
        composable_node_descriptions= [
            ComposableNode(
                package="kr_trackers_manager",
                plugin="TrackersManager",
                name="trackers_manager",
                parameters=[config_file],
            ),
            ComposableNode(
                package="kr_trackers_manager",
                plugin="TrackersManagerLifecycleManager",
                name="lifecycle_manager",
                parameters=[
                    {'node_name': "trackers_manager"}
                ]
            ),
            ComposableNode(
                package="kr_mav_controllers",
                plugin="SO3ControlComponent",
                name="so3_controller",
            ),
        ],
        output='screen'
    )
    

    return LaunchDescription([
        robot_arg,
        config_file_arg,
        OpaqueFunction(function=print_config_file),  # Call the function to print config_file
        container,
    ])
