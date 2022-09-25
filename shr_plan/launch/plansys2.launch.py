from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    shr_dir = get_package_share_directory('shr_plan')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py'])),
        launch_arguments={
            'model_file': PathJoinSubstitution([shr_dir, 'pddl', 'domain_shr.pddl']),
            'namespace': namespace
        }.items())

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    # Declare the launch options
    ld.add_action(plansys2_cmd)

    # actions
    move_cmd = Node(
        package='plansys2_simple_example',
        executable='move_action_node',
        name='move_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    charge_cmd = Node(
        package='plansys2_simple_example',
        executable='charge_action_node',
        name='charge_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    ask_charge_cmd = Node(
        package='plansys2_simple_example',
        executable='ask_charge_action_node',
        name='ask_charge_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])  # Create the launch description and populate


    # ld.add_action(move_cmd)
    # ld.add_action(charge_cmd)
    # ld.add_action(ask_charge_cmd)

    return ld
