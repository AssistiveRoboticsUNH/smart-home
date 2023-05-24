from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    smartthings_node = Node(
        package='smartthings_ros',
        executable='smartthings_node',
        output='screen'
    )

    nav_bridge_cmd = Node(
        package='shr_actions_py',
        executable='nav2_zmq_action',
        name='nav2_zmq_action',
        output='screen')

# TO DO ADD THE CAMERA TOPIC AS AN ARGUEMNT
    # launched in world_state in plansys2
#     yolo_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(PathJoinSubstitution([
#             get_package_share_directory('yolostate'), 'launch', 'detecthuman.launch.py']))
#     )

    tf_broadcast = Node(
        package='shr_plan',
        executable='tf_broadcaster_node',
    )

    ld = LaunchDescription()
    ld.add_action(nav_bridge_cmd)
    ld.add_action(tf_broadcast)
    ld.add_action(smartthings_node)
    # ld.add_action(yolo_cmd)
    return ld
