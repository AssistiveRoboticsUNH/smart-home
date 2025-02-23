import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    docking_camera_server_cmd = Node(
        package='shr_docking',
        executable='docking_camera_server',
        name='docking_camera_server',
        output='screen')

    docking_ir_server_cmd = Node(
        package='shr_docking',
        executable='docking_ir_server',
        name='docking_ir_server',
        output='screen')
    
    # Dokcing server with camera and infrared combined
    docking_server_cmd = Node(
        package='shr_docking',
        executable='docking_server',
        name='docking_server',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(docking_camera_server_cmd)
    ld.add_action(docking_ir_server_cmd)
    ld.add_action(docking_server_cmd)

    return ld
