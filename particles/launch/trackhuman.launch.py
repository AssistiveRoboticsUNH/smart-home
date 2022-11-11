from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
import os



def generate_launch_description():
    ld = LaunchDescription()


    run_hello_particles = Node(
        package="particles",
        executable="hello",
        name="hello", 
        output="log"
    )

    ld.add_action(run_hello_particles)

    return ld
