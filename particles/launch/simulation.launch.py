    # path = get_package_share_directory('pioneer_description')
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(path, 'launch', 'robot_state_publisher.launch.py'))))

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    unity_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('shr_plan'), 'launch', 'simulation.launch.py']))
    )

    yolo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('yolostate'), 'launch', 'detecthuman.launch.py']))
    )

    ld = LaunchDescription()
    ld.add_action(unity_sim_cmd)
    ld.add_action(yolo_cmd)

    return ld