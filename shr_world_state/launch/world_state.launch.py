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

    world_state_node = Node(
        package="shr_world_state",
        executable="world_state_node",
        name="world_state_node",
        output="log"
    )

    ld.add_action(world_state_node)

    detect_eating_sim_node = Node(
        package="shr_world_state",
        executable="detect_eating_sim_node",
        name="detect_eating_sim_node",
        output="log"
    )

    ld.add_action(detect_eating_sim_node)

    detect_taking_pill_sim_node = Node(
        package="shr_world_state",
        executable="detect_taking_pill_sim_node",
        name="detect_taking_pill_sim_node",
        output="log"
    )

    ld.add_action(detect_taking_pill_sim_node)

    detect_bed_after_returning_node = Node(
        package="shr_world_state",
        executable="detect_bed_after_returning_node",
        name="detect_bed_after_returning_node",
        output="log"
    )

    ld.add_action(detect_bed_after_returning_node)

    yolostate_path = get_package_share_directory('yolostate')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yolostate_path, 'launch', 'detecthuman.launch.py'))))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yolostate_path, 'launch', 'detecthumaninhouse.launch.py'))))

    return ld
