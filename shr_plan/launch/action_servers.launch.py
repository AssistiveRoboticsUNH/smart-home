import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    docking_launch_file_path = os.path.join(
        get_package_share_directory('shr_docking'),
        'launch',
        'docking_action_servers.launch.py'
    )

    docking_launch_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(docking_launch_file_path)
        )
    
    # # read script
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

    localize_cmd = Node(
        package='shr_actions_py',
        executable='localize',
        name='localize',
        output='screen')

    undock_cmd = Node(
        package='shr_actions_py',
        executable='undocking',
        name='undock',
        output='screen')

    play_audio_text_node_cmd = Node(
        package='shr_actions_py',
        executable='play_audio_text_action',
        name='play_audio_text_action',
        output='screen')

    

    ld = LaunchDescription()
    #ld.add_action(read_script_node_cmd)
    #ld.add_action(play_audio_node_cmd)
    # ld.add_action(play_video_node_cmd)
    ld.add_action(make_call_node_cmd)
    #ld.add_action(send_text_node_cmd)
    ld.add_action(localize_cmd)
    ld.add_action(undock_cmd)
    ld.add_action(play_audio_text_node_cmd)
    #ld.add_action(waypoint_cmd)
    ld.add_action(docking_launch_cmd)
    # ld.add_action(ask_question_cmd)

    return ld
