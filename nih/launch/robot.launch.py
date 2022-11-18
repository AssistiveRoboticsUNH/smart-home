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


    p2os_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('p2os_driver'), 'launch', 'p2os_driver.launch.py'])),
        remappings=[
            ('/pose', '/odom'),
        ]
    )
    ld.add_action(p2os_cmd)

    sick_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('sick_scan'), 'launch', 'sick_lms_5xx.launch.py'])),
        remappings=[
            ('/sick_lms_5xx/scan', '/scan'),
        ]
    )
    ld.add_action(sick_cmd)

    pub_motor_cmd= Node(
        package="nih",
        executable="pub_motor",
        name="pub_motor",
        output="log"
    )
    ld.add_action(pub_motor_cmd)


    return ld
