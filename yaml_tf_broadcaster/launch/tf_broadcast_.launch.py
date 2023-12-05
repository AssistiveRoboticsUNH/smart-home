from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    pkg_path = get_package_share_directory('yaml_tf_broadcaster') + "/config/"

    aptags_file = DeclareLaunchArgument(
        "aptags_location",
        default_value=pkg_path + "aptags_location.yaml",
        description="aptags location"
    )
    ld.add_action(aptags_file)
    aptags = Node(
        package="yaml_tf_broadcaster",
        executable="yaml_broadcaster_node",
        name="aptags_launch_",
        parameters=[
            {"yaml_file_name": LaunchConfiguration("aptags_location")}
        ]
    )

    ld.add_action(aptags)



    # tf_mat_file = DeclareLaunchArgument(
    #     "tf_mat_Lo",
    #     default_value=pkg_path + "transformation_matrix.yaml",
    #     description="tf_mat"
    # )
    #
    # ld.add_action(tf_mat_file)
    #
    # tf_mat = Node(
    #     package="aptags_tf_broadcast",
    #     executable="aptag_broadcast_node",
    #     name="tf_mat",
    #     parameters=[
    #         {"yaml_file_name": LaunchConfiguration("tf_mat_Lo")}
    #     ]
    # )
    #
    # ld.add_action(tf_mat)
    return ld
