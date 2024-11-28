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
    tb_unizar_nav_dir = get_package_share_directory('tb_unizar_nav')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

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
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if True',
    )

    # Locate description TODO: Fix the xacro compilation  https://gist.github.com/clalancette/5d15df1f54a1e01946659dbfa6c46c30
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
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)


    return LaunchDescription([
        declarenamespace_cmd,
        declare_use_namespace_cmd,
        declare_use_sim_time_cmd,

        start_robot_state_publisher_cmd
    ])
