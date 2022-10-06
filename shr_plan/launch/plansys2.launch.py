from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration('use_simulation')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    declare_use_sim_cmd = DeclareLaunchArgument(
        'use_simulation',
        default_value='True',
        description='Use simulation')

    shr_dir = get_package_share_directory('shr_plan')
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py'])),
        launch_arguments={
            'model_file': PathJoinSubstitution([shr_dir, 'pddl', 'domain_shr.pddl']),
            'namespace': namespace
        }.items())

    unity_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('unity_launch'), 'launch', 'launch.launch.py'])),
        condition=IfCondition(use_sim)
    )

    pioneer_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('pioneer_navigation2'), 'launch', 'navigation2.launch.py']))
    )

    sound_node_cmd = Node(
        package='sound_play',
        executable='soundplay_node.py',
        name='soundplay_node',
        output='screen')

    face_node_cmd = Node(
        package='pioneer_shr_py',
        executable='recognize_face_action',
        name='recognize_face_action',
        output='screen')

    read_script_node_cmd = Node(
        package='pioneer_shr_py',
        executable='read_script_action',
        name='read_script_action',
        output='screen')

    play_audio_node_cmd = Node(
        package='pioneer_shr_py',
        executable='play_audio_action',
        name='play_audio_action',
        output='screen')

    play_video_node_cmd = Node(
        package='pioneer_shr_py',
        executable='play_video_action',
        name='play_video_action',
        output='screen')

    open_image_node_cmd = Node(
        package='pioneer_shr_py',
        executable='open_image_action',
        name='open_image_action',
        output='screen')

    make_call_node_cmd = Node(
        package='pioneer_shr_py',
        executable='make_call_action',
        name='make_call_action',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_cmd)
    ld.add_action(pioneer_nav_cmd)

    ld.add_action(plansys2_cmd)
    ld.add_action(unity_sim_cmd)
    ld.add_action(sound_node_cmd)
    ld.add_action(face_node_cmd)
    ld.add_action(read_script_node_cmd)
    ld.add_action(play_audio_node_cmd)
    ld.add_action(play_video_node_cmd)
    ld.add_action(open_image_node_cmd)
    ld.add_action(make_call_node_cmd)

    return ld
