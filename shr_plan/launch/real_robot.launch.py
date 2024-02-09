from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    ld = LaunchDescription()

    smartthings_node = Node(
        package='smartthings_ros',
        executable='smartthings_node',
        output='screen'
    )
    smartthings_node_plug = Node(
        package='smartthings_ros',
        executable='smartplug_node',
        output='screen'
    )
    smartthings_initial_pose = Node(
        package='smartthings_ros',
        executable='initial_pose',
        output='screen'
    )

    protocol_time_node = Node(
        package='shr_plan',
        executable='time_publisher_node',
        output='screen'
    )

    logger_node = Node(
        package='simple_logger',
        executable='simple_logger_web',
        output='screen'
    )

    # nav_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([
    #         get_package_share_directory('jackal_navigation'), 'launch', 'navigation2_jackal.launch.py']))
    # )

    # realsense_cam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([
    #         get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py']))
    # )

    # jackal_navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([
    #         get_package_share_directory('jackal_navigation'), 'launch', 'navigation2_jackal.launch.py']))
    # )

    apriltags_realsense_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('apriltag_ros'), 'launch', 'tag_realsense_loc.launch.py']))
    )


    tf_broadcast = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('yaml_tf_broadcaster'), 'launch', 'tf_broadcast.launch.py']))
    )


    charger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('charger_description'), 'launch', 'view_charger.launch.py']))
    )

    #     package='shr_plan
    #
    #     # planner_cmd = Node(',
    #     executable='planning_controller_node',
    #     output='screen'
    # )
    gpu = False
    if gpu:
        apriltags_zed = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                get_package_share_directory('apriltag_ros'), 'launch', 'tag_zed.launch.py']))
        )
        ld.add_action(apriltags_zed)

        ## zed wrapper
        zed_wrapper = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                get_package_share_directory('zed_wrapper'), 'launch', 'zed2i.launch.py']))
        )
        ld.add_action(zed_wrapper)

        ## particle filter
        # particle_filter = Node(
        #     package='particle_filter',
        #     executable='particle_filter_node',
        #     output='screen'
        # )
        # ld.add_action(particle_filter)

# ld.add_action(planner_cmd)
#     ld.add_action(nav_cmd)
    # ld.add_action(logger_node)
    # ld.add_action(jackal_navigation)
    # ld.add_action(realsense_cam)
    # ld.add_action(apriltags_realsense_docking)
    ld.add_action(charger)
    ld.add_action(apriltags_realsense_loc)
    ld.add_action(tf_broadcast)
    ld.add_action(smartthings_node)
    # ld.add_action(smartthings_node_plug)
    # ld.add_action(protocol_time_node)

    return ld
