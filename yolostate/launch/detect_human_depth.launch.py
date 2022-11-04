from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    ld = LaunchDescription()

    download_weights_if_necessary = Node(
        package="yolostate",
        executable="downloadyolo",
        name="downloadyolo",
        output="log"
    )
    run_detect_human_depth = Node(
        package="yolostate",
        executable="detect_human_depth",
        name="detect_human_depth",
        parameters=[{"view_camera": False, "view_depth_camera": True}], 
        output="log"
    )

    ld.add_action(download_weights_if_necessary)
    ld.add_action(run_detect_human_depth)

    return ld
