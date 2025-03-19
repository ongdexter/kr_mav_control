
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare

def parse_yaml(context):
    # Load the crazyflies YAML file
    crazyflies_yaml = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    with open(crazyflies_yaml, 'r') as file:
        crazyflies = yaml.safe_load(file)

    # server params
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_params = [crazyflies] + [server_yaml_content['/crazyflie_server']['ros__parameters']]
    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    server_params[1]['robot_description'] = robot_desc


    return [
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            condition=LaunchConfigurationEquals('backend','cflib'),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','cpp'),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
            prefix=PythonExpression(['"xterm -e gdb -ex run --args" if ', LaunchConfiguration('debug'), ' else ""']),
        ),
        Node(
            package='crazyflie_sim',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','sim'),
            name='crazyflie_server',
            output='screen',
            emulate_tty=True,
            parameters= server_params,
        )]

def generate_launch_description():
    # Crazyflie default paths
    default_crazyflies_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')


    default_rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'config.rviz')

    telop_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop.yaml')

    # KR interface paths & configurations
    config_file = FindPackageShare('kr_crazyflie_interface').find('kr_crazyflie_interface') + '/config/crazyflie.yaml'
    trackers_manager_config_file = FindPackageShare('kr_trackers_manager').find('kr_trackers_manager') + '/config/trackers_manager.yaml'
    trackers_manager_launch_path = os.path.join(
        FindPackageShare('kr_trackers_manager').find('kr_trackers_manager'),
        'launch',
        'example.launch.py'
    )

    # Define launch arguments
    # Original Crazyflie arguments
    crazyflie_args = [
        DeclareLaunchArgument('crazyflies_yaml_file',
                              default_value=default_crazyflies_yaml_path),
        DeclareLaunchArgument('rviz_config_file',
                              default_value=default_rviz_config_path),
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('debug', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='False'),
        DeclareLaunchArgument('gui', default_value='True'),
        DeclareLaunchArgument('teleop', default_value='False'),
        DeclareLaunchArgument('mocap', default_value='True'),
        DeclareLaunchArgument('teleop_yaml_file', default_value=''),
        # Adding mocap vicon specific parameters
        DeclareLaunchArgument('mocap_server', default_value='mocap.perch'),
        DeclareLaunchArgument('mocap_frame_rate', default_value='100'),
        DeclareLaunchArgument('mocap_max_accel', default_value='10.0'),
    ]

    # KR interface arguments
    kr_args = [
        DeclareLaunchArgument('robot', default_value='cf3'),
        DeclareLaunchArgument('odom', default_value='odom'),
        DeclareLaunchArgument('so3_cmd', default_value='so3_cmd'),
    ]

    # Initialize launch description with all arguments
    ld = LaunchDescription(crazyflie_args + kr_args)

    # Add the original Crazyflie nodes
    ld.add_action(OpaqueFunction(function=parse_yaml))

    # Add mocap_vicon node
    ld.add_action(
        Node(
            condition=IfCondition(LaunchConfiguration('mocap')),
            package='mocap_vicon',
            executable='mocap_vicon_node',
            name='vicon',
            output='screen',
            parameters=[
                {'server_address': LaunchConfiguration('mocap_server')},
                {'frame_rate': LaunchConfiguration('mocap_frame_rate')},
                {'max_accel': LaunchConfiguration('mocap_max_accel')},
                {'publish_tf': False},
                {'publish_pts': False},
                {'fixed_frame_id': 'mocap'},
                # Set to [''] to take in ALL models from Vicon
                {'model_list': ['cf3']},
            ],
            remappings=[
                # Uncomment and modify the remapping if needed
                # ('vicon/model_name/odom', '/model_name/odom'),
            ]
        )
    )

    # ld.add_action(
    #     Node(
    #         condition=LaunchConfigurationEquals('rviz', 'True'),
    #         package='rviz2',
    #         namespace='',
    #         executable='rviz2',
    #         name='rviz2',
    #         arguments=['-d', LaunchConfiguration('rviz_config_file')],
    #         parameters=[{
    #             "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
    #         }]
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         condition=LaunchConfigurationEquals('gui', 'True'),
    #         package='crazyflie',
    #         namespace='',
    #         executable='gui.py',
    #         name='gui',
    #         parameters=[{
    #             "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
    #         }]
    #     )
    # )

    # Add the KR interface nodes
    ld.add_action(
        Node(
            package="kr_mav_manager",
            executable="mav_services",
            namespace=LaunchConfiguration('robot'),
            name="mav_services",
            output='screen',
        )
    )

    ld.add_action(
        Node(
            package="rqt_mav_manager",
            executable="rqt_mav_manager",
            namespace=LaunchConfiguration('robot'),
            name="rqt_mav",
            output='screen',
        )
    )

    # Include the trackers manager launch file with namespace
    trackers_component = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(trackers_manager_launch_path),
        launch_arguments={
            'config_file': trackers_manager_config_file
        }.items()
    )

    trackers_component_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('robot')),
            trackers_component,
        ]
    )

    ld.add_action(trackers_component_with_namespace)

    # Add the SO3 command to Crazyflie component container
    ld.add_action(
        ComposableNodeContainer(
            name="so3_container",
            namespace=LaunchConfiguration('robot'),
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="kr_crazyflie_interface",
                    plugin="SO3CmdToCrazyflie",
                    name="so3cmd_to_crazyflie",
                    parameters=[config_file],
                    remappings=[
                        ("~/odom", LaunchConfiguration('odom')),
                        ("~/so3_cmd", LaunchConfiguration('so3_cmd'))
                    ]
                )
            ],
            output='screen'
        )
    )

    return ld