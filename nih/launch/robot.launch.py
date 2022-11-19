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

    parameters = [{"usb_port": "/dev/ttyUSB0"}]
    p2os_node= Node(
        package='p2os_driver',
        executable='p2os_driver',
        parameters=parameters,
        remappings=[
            ('/pose', '/odom'),
        ]
    )
    ld.add_action(p2os_node)

    # node_arguments=['launch/sick_lms_5xx.launch']  #TODO: check if works.
    #
    # ROS_DISTRO = os.environ.get('ROS_DISTRO') # i.e. 'eloquent', 'foxy', etc.
    # if ROS_DISTRO[0] <= "e": # ROS versions eloquent and earlier require "node_executable", ROS foxy and later use "executable"
    #     node = Node(
    #         package='sick_scan',
    #         node_executable='sick_generic_caller',
    #         output='screen',
    #         arguments=node_arguments
    #     )
    # else: # ROS versions eloquent and earlier require "node_executable", ROS foxy and later use "executable"
    #     node = Node(
    #         package='sick_scan',
    #         executable='sick_generic_caller',
    #         output='screen',
    #         arguments=node_arguments
    #     )
    #
    # ld.add_action(node)
    # return ld

    # sick_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([
    #         get_package_share_directory('sick_scan'), 'launch', 'sick_lms_5xx.launch.py'])),
    #     # remappings=[
    #     #     ('/sick_lms_5xx/scan', '/scan'),
    #     # ]
    # )
    # ld.add_action(sick_cmd)

    pub_motor_cmd= Node(
        package="nih",
        executable="pub_motor",
        name="pub_motor",
        output="log"
    )
    ld.add_action(pub_motor_cmd)


    return ld
