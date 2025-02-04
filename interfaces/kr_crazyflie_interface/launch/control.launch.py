from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

  robot_ns = LaunchConfiguration('robot')  
  odom_topic = LaunchConfiguration('odom')
  so3_cmd_topic = LaunchConfiguration('so3_cmd')

  robot_arg = DeclareLaunchArgument(
    'robot', default_value=''
  )
  odom_arg = DeclareLaunchArgument(
    'odom', default_value='odom'
  )
  so3_cmd_arg = DeclareLaunchArgument(
    'so3_cmd', default_value='so3_cmd'
  )

  # Path to the configuration file
  config_file = FindPackageShare('kr_crazyflie_interface').find('kr_crazyflie_interface') + '/config/crazyflie.yaml'
  
  manager_node = Node(
      package="kr_mav_manager",
      executable="mav_services",
      namespace=LaunchConfiguration('robot'),
      name="manager",
      output='screen'
  )

  rqt_gui_node = Node(
      package="rqt_mav_manager",
      executable="rqt_mav_manager",
      namespace=LaunchConfiguration('robot'),
      name="rqt_mav",
      output='screen'
  )

  trackers_component = ComposableNodeContainer(
    name="trackers_container",
    namespace=LaunchConfiguration('robot'),
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[
      ComposableNode(
        package="kr_trackers_manager",
        plugin="TrackersManager",
        name="kr_trackers_manager",
      ),
      ComposableNode(
        package="kr_trackers_manager",
        plugin="TrackersManagerLifecycleManager",
        name="kr_trackers_manager_lifecycle",
      ),
      ComposableNode(
        package="kr_mav_controllers",
        plugin="SO3ControlComponent",
        name="so3_controller",
      ),
    ],
    output='screen'
  )
  # EXAMPLE BELOW
  # Component configuration
  # so3cmd_to_crazyflie_component = ComposableNodeContainer(
  #   name="so3_container",
  #   namespace=LaunchConfiguration('robot'),
  #   package="rclcpp_components",
  #   executable="component_container",
  #   composable_node_descriptions=[
  #     ComposableNode(
  #       package="kr_crazyflie_interface",
  #       plugin="SO3CmdToCrazyflie",
  #       name="so3cmd_to_crazyflie",
  #       parameters=[config_file],
  #       remappings=[
  #         ("~/odom", odom_topic),
  #         ("~/so3_cmd", so3_cmd_topic)
  #       ]
  #     )
  #   ],
  #   output='screen'
  # )
    # Regular Node (not in container)


  return LaunchDescription([
      robot_arg,
      odom_arg,
      so3_cmd_arg,
      manager_node,
      rqt_gui_node,
      trackers_component,

  ])
