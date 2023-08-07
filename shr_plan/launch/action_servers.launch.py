import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # read script with deep fake
    read_script_node_cmd = Node(
        package='shr_actions_py',
        executable='read_script_action',
        name='read_script_action',
        output='screen')

    play_audio_node_cmd = Node(
        package='shr_actions_py',
        executable='play_audio_action',
        name='play_audio_action',
        output='screen')

    play_video_node_cmd = Node(
        package='shr_actions_py',
        executable='play_video_action',
        name='play_video_action',
        output='screen')

    open_image_node_cmd = Node(
        package='shr_actions_py',
        executable='open_image_action',
        name='open_image_action',
        output='screen')

    make_call_node_cmd = Node(
        package='shr_actions_py',
        executable='make_call_action',
        name='make_call_action',
        output='screen')

    send_text_node_cmd = Node(
        package='shr_actions_py',
        executable='send_text_action',
        name='send_text_action',
        output='screen')

    rotate_node_cmd = Node(
        package='shr_actions_py',
        executable='rotate_action',
        name='rotate_action',
        output='screen')

    detect_person_cmd = Node(
        package='shr_actions_py',
        executable='detect_person_action',
        name='detect_person_action',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(read_script_node_cmd)
    ld.add_action(play_audio_node_cmd)
    ld.add_action(play_video_node_cmd)
    ld.add_action(open_image_node_cmd)
    ld.add_action(make_call_node_cmd)
    ld.add_action(send_text_node_cmd)
    ld.add_action(rotate_node_cmd)
    ld.add_action(detect_person_cmd)

    return ld