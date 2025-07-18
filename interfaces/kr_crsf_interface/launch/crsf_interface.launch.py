import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Declare launch arguments
    robot_ns   = LaunchConfiguration('robot',   default='/')
    odom_topic = LaunchConfiguration('odom',    default='odom')
    so3_cmd_topic  = LaunchConfiguration('so3_cmd', default='so3_cmd')

    # Path to the configuration file
    config_file = os.path.join(
        get_package_share_directory('kr_crsf_interface'),
        'config',
        'neurofly.yaml'
    )

    # Component container to load your SO3CmdToCRSF component
    crsf_container = ComposableNodeContainer(
        name='so3cmd_to_crsf_container',
        namespace=robot_ns,
        package='rclcpp_components',
        executable='component_container_mt',  # multi-threaded
        composable_node_descriptions=[
            ComposableNode(
                package='kr_crsf_interface',
                plugin='SO3CmdToCRSF',
                name='so3cmd_to_crsf',
                remappings=[
                    ('odom',   odom_topic),
                    ('so3_cmd', so3_cmd_topic),
                ],
                parameters=[config_file],
            ),
        ],
        output='screen',
        )
    robot_arg = DeclareLaunchArgument(
        'robot', default_value=''
    )
    odom_arg = DeclareLaunchArgument(
        'odom', default_value='odom'
    )
    so3_cmd_arg = DeclareLaunchArgument(
        'so3_cmd', default_value='so3_cmd'
    )
    return LaunchDescription([
        robot_arg,
        odom_arg,
        so3_cmd_arg,
        crsf_container,
    ])
