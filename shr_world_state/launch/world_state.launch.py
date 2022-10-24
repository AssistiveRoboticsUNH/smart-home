from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os


def generate_launch_description():
    ld = LaunchDescription()

    world_state_node = Node(
        package="shr_world_state",
        executable="world_state_node",
        name="world_state_node",
        output="log"
    )

    ld.add_action(world_state_node)

    yolostate_path = get_package_share_directory('yolostate')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yolostate_path, 'launch', 'detecthuman.launch.py'))))

    return ld
