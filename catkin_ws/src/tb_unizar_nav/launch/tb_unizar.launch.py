import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Paths to bringup and custom package
    bringup_dir = get_package_share_directory('nav2_bringup')
    tb_unizar_nav_dir = get_package_share_directory('tb_unizar_nav')

    # Paths to launch and params files
    bringup_launch_path = os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
    default_nav_params_file = os.path.join(tb_unizar_nav_dir, 'params', 'nav2_params.yaml')
    default_kobuki_params_file = os.path.join(tb_unizar_nav_dir, 'params', 'kobuki_params.yaml')
    default_map_file = os.path.join(tb_unizar_nav_dir, 'maps', 'dummy.yaml')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_params_file = LaunchConfiguration('nav_params_file')
    kobuki_params_file = LaunchConfiguration('kobuki_params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_localization = LaunchConfiguration('use_localization')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Declare launch arguments
    declarenamespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='ugv0', description='Namespace for the robot'
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack',
    )
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=default_map_file, description='Full path to map yaml file to load'
    )
    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='False',
        description='Whether to enable localization or not'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if True',
    )
    declare_nav_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=default_nav_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_kobuki_params_file_cmd = DeclareLaunchArgument(
        'kobuki_params_file',
        default_value=default_kobuki_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )


    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    
    declare_kobuki_ros_node = Node(package='kobuki_node',
                                  executable='kobuki_ros_node',
                                  output='both',
                                  parameters=[kobuki_params_file])

    # Locate description TODO: Fix the xacro compilation
    urdf = os.path.join(tb_unizar_nav_dir, 'urdf', 'compiled_turtlebot_unizar.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # Group the kobuki_node launch file under the specified namespace
    # kobuki_group = GroupAction(
    #     actions=[
    #         PushRosNamespace(namespace),  # Apply the namespace dynamically
    #         declare_kobuki_ros_node,
    #     ]
    # )

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)


    # Include the bringup launch file
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_localization': use_localization,
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'log_level': log_level
        }.items(),
    )

    return LaunchDescription([
        # Declare launch arguments
        declarenamespace_cmd,
        declare_use_namespace_cmd,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_use_localization_cmd,
        declare_use_sim_time_cmd,
        declare_nav_params_file_cmd,
        declare_kobuki_params_file_cmd,        
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        declare_use_robot_state_pub_cmd,
        # Include the kobuki group
        declare_kobuki_ros_node,
        start_robot_state_publisher_cmd,
        # Include the bringup launch file
        bringup_launch,
    ])
