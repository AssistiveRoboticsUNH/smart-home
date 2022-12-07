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

    # run the pioneer driver
    parameters = [{"usb_port": "/dev/ttyUSB0"}]
    p2os_node = Node(
        package='p2os_driver',
        executable='p2os_driver',
        parameters=parameters,
        remappings=[
            ('/pose', '/odom'),
        ]
    )
    ld.add_action(p2os_node)

    # publish motor state 1 so that the robot doesn't beep
    pub_motor_cmd = Node(
        package="nih",
        executable="pub_motor",
        name="pub_motor",
        output="log"
    )
    ld.add_action(pub_motor_cmd)

    # Important: without it rviz can't show robot model and laser data
    robot_state_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('pioneer_description'), 'launch', 'robot_state_publisher.launch.py'])),
    )
    ld.add_action(robot_state_cmd)


    # static_trans_cmd = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map',
    #                '--child-frame-id', 'odom']
    # )
    # ld.add_action(static_trans_cmd)

    # TODO: static transform publisher
    # odom frame map frame

    return ld
