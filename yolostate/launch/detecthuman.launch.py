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

    download_weights_if_necessary= Node(
        package="yolostate",
        executable="downloadyolo",
        name="downloadyolo",
        output="log"
    )
    run_detecthuman= Node(
        package="yolostate",
        executable="detecthuman",
        name="detecthuman",
        output="log"
    )

    ld.add_action(download_weights_if_necessary)
    ld.add_action(run_detecthuman)

    return ld
    