from launch.event_handlers.on_process_exit import OnProcessExit
import xacro
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_path,
)


package_name = "z1_tsid"
pkg_prefix_dir = get_package_prefix(package_name)
pkg_share_dir = get_package_share_path(package_name)

def launch_setup(context, *args, **kwargs):

    # z1_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         FindPackageShare("z1_description"), "/launch/z1_spawner.launch.py"
    #     ], ),
    #     launch_arguments={"controller_config": os.path.join(pkg_share_dir, "config", "z1_controllers.yaml"),
    #                       "starting_controller": "tsid_controller"}.items(),
    # )

    # marker_node = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["motion_control_handle", "-c", "/controller_manager"],
    #         )
    marker_node = Node(
            package="motion_control_handle",
            executable="pose_handle",
            )
    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                os.path.join(get_package_share_path("motion_control_handle"), "rviz", "pose_handle.rviz"),
                ],
            )

    nodes_to_start = [
            # z1_description,
            rviz_node,
            marker_node,
    ]
    return nodes_to_start



def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
