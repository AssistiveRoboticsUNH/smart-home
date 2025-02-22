from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file_path = os.path.join( get_package_share_directory('convros_bot'), 'config', 'config.yaml')

    return LaunchDescription([

        # Action sever node to retuen response to questions passed as goal 
        Node(
            package='convros_bot',
            executable='question_response_action',
            name='question_response_action',
            output='screen',
            parameters=[config_file_path]
        ),
        

    ])
