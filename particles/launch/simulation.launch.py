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
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    unity_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('shr_plan'), 'launch', 'simulation.launch.py']))
    )

    yolo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            # get_package_share_directory('yolostate'), 'launch', 'detecthuman.launch.py']))
            get_package_share_directory('yolostate'), 'launch', 'detect_human_depth.launch.py']))
    )



    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("particles"), "rviz", "trackhuman.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


    ld = LaunchDescription()
    ld.add_action(unity_sim_cmd)
    ld.add_action(yolo_cmd)
    ld.add_action(rviz_node)

    return ld
