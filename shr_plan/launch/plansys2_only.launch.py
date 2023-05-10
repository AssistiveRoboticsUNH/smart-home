import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    shr_dir = get_package_share_directory('shr_plan')
    params_file = os.path.join(shr_dir, 'params', 'plansys2_params.yaml')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py'])),
        # 'plansys2_bringup_launch_monolithic.py' 'plansys2_bringup_launch_distributed.py'
        launch_arguments={
            'model_file': PathJoinSubstitution([shr_dir, 'pddl', 'food_domain.pddl']),
            'namespace': namespace,
            'bt_builder_plugin': 'ContingentBTBuilder',
            'params_file': params_file,
        }.items())

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(plansys2_cmd)

    return ld
