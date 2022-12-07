from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    ld = LaunchDescription()

    pioneer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('nih'), 'launch', 'pioneer.launch.py'])),
    )
    ld.add_action(pioneer_cmd)

    sick_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('nih'), 'launch', 'sick_lms_5xx.launch.py'])),
    )
    ld.add_action(sick_cmd)

    return ld
