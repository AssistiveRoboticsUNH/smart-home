from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    unity_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('unity_launch'), 'launch', 'launch_jackal.launch.py']))
    )

    pioneer_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('jackal_navigation'), 'launch', 'navigation2_jackal.launch.py']))
    )

    ld = LaunchDescription()

    ld.add_action(unity_sim_cmd)
    ld.add_action(pioneer_nav_cmd)

    return ld
