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
    run_detecthuman = Node(
        package="yolostate",
        executable="detecthuman",
        name="detecthuman",
        output="log"
    )

    ld.add_action(download_weights_if_necessary)
    ld.add_action(run_detecthuman)

    return ld
