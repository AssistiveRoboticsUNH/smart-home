from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    shr_dir = get_package_share_directory('shr_plan')
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_distributed.py'])),
        # 'plansys2_bringup_launch_monolithic.py' 'plansys2_bringup_launch_distributed.py'
        launch_arguments={
            'model_file': PathJoinSubstitution([shr_dir, 'pddl', 'paul_domain_shr_conditional.pddl']),
            'namespace': namespace,
            'bt_builder_plugin': 'ContingentBTBuilder',
        }.items())

    sound_node_cmd = Node(
        package='sound_play',
        executable='soundplay_node.py',
        name='soundplay_node',
        output='screen')

    face_node_cmd = Node(
        package='shr_actions_py',
        executable='recognize_face_action',
        name='recognize_face_action',
        output='screen')

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

    open_image_node_cmd = Node(
        package='shr_actions_py',
        executable='open_image_action',
        name='open_image_action',
        output='screen')

    make_call_node_cmd = Node(
        package='shr_actions_py',
        executable='make_call_action',
        name='make_call_action',
        output='screen')

    rotate_node_cmd = Node(
        package='shr_actions_py',
        executable='rotate_action',
        name='rotate_action',
        output='screen')

    find_person_cmd = Node(
        package='shr_actions_cpp',
        executable='find_person_node',
        name='find_person_node',
        output='screen')

    world_state_cmd = Node(
        package='shr_world_state',
        executable='world_state_node',
        name='world_state_node',
        output='screen')

    # plansys2 actions
    moveto_landmark_cmd = Node(
        package='shr_plan',
        executable='move_action_node',
        name='move_action_node',
        output='screen')

    notify_automated_cmd = Node(
        package='shr_plan',
        executable='notify_automated_action_node',
        name='notify_automated_action_node',
        output='screen')

    notify_recorded_video_cmd = Node(
        package='shr_plan',
        executable='notify_recorded_video_node',
        name='notify_recorded_video_node',
        output='screen')

    planning_controller_node_cmd = Node(
        package='shr_plan',
        executable='planning_controller_node',
        name='planning_controller_node',
        output='screen')

    call_node_cmd = Node(
        package='shr_plan',
        executable='call_action_node',
        name='call_action_node',
        output='screen')

    node_action_node_cmd = Node(
        package='shr_plan',
        executable='none_action_node',
        name='none_action_node',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)

    ld.add_action(plansys2_cmd)
    ld.add_action(sound_node_cmd)
    ld.add_action(face_node_cmd)
    ld.add_action(read_script_node_cmd)
    ld.add_action(play_audio_node_cmd)
    ld.add_action(play_video_node_cmd)
    ld.add_action(open_image_node_cmd)
    ld.add_action(make_call_node_cmd)
    ld.add_action(rotate_node_cmd)
    ld.add_action(find_person_cmd)
    ld.add_action(world_state_cmd)

    ld.add_action(moveto_landmark_cmd)
    ld.add_action(notify_automated_cmd)
    ld.add_action(notify_recorded_video_cmd)
    ld.add_action(call_node_cmd)
    ld.add_action(node_action_node_cmd)

    ld.add_action(planning_controller_node_cmd)

    return ld
