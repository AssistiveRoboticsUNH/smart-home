from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file_path = os.path.join( get_package_share_directory('convros_bot'), 'config', 'config.yaml')

    return LaunchDescription([
        # Node handling Speech to Text node
        Node(
            package='convros_bot',
            executable='stt_node',
            name='stt_node',
            output='screen'
        ),

        # Node handling Text to Speech node
        Node(
            package='convros_bot',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[config_file_path]
        ),

        # Action sever node to handle action client requests forwarded from conversation handler(speech_processor) nodespeech_processor
        Node(
            package='convros_bot',
            executable='conversation_action',
            name='conversation_action',
            output='screen',
            parameters=[config_file_path]
        ),

        # Node handling Speech processor node to look for wake up call and call action server
        Node(
            package='convros_bot',
            executable='speech_processor',
            name='speech_processor',
            output='screen',
            parameters=[config_file_path]
        ),

        

    ])
