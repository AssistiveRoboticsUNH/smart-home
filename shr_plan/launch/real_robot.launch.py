from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    smartthings_node = Node(
        package='smartthings_ros',
        executable='smartthings_node',
        output='screen'
    )
    #
    # logger_node = Node(
    #     package='simple_logger',
    #     executable='simple_logger_web',
    #     output='screen'
    # )

    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('jackal_navigation'), 'launch', 'navigation2_jackal.launch.py']))
    )

    apriltags = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('apriltag_ros'), 'launch', 'tag_realsense.launch.py']))
    )

    tf_broadcast = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('aptags_tf_broadcast'), 'launch', 'tf_broadcast.launch.py']))
    )

    # planner_cmd = Node(
    #     package='shr_plan',
    #     executable='planning_controller_node',
    #     output='screen'
    # )

    #

    ld = LaunchDescription()
    # ld.add_action(planner_cmd)
    ld.add_action(nav_cmd)
    # ld.add_action(logger_node)
    ld.add_action(apriltags)
    ld.add_action(tf_broadcast)
    ld.add_action(smartthings_node)

    return ld
