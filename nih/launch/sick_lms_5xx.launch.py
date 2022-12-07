import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan')
    launchfile = os.path.basename(__file__)[:-3] # convert "<lidar_name>.launch.py" to "<lidar_name>.launch"
    launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/' + launchfile) # 'launch/sick_lms_5xx.launch')
    node_arguments=[launch_file_path]

    node = Node(
        package='sick_scan',
        executable='sick_generic_caller',
        output='screen',
        arguments=node_arguments,
        remappings=[
            ('/sick_lms_5xx/scan', '/scan'),
        ]
    )
    ld.add_action(node)
    return ld

