"""Launch file for the relative position finder node.

This launch file starts "yolo_node" and "pos_finder_node" when invoked."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
)

from launch_ros.actions import Node


def generate_launch_description():
    debug_enabled_launch_arg = DeclareLaunchArgument(
        "debug_enabled", default_value=TextSubstitution(text="False")
    )
    camera_fov_launch_arg = DeclareLaunchArgument(
        "camera_fov", default_value=TextSubstitution(text="3.089")
    )
    camera_width_launch_arg = DeclareLaunchArgument(
        "camera_width", default_value=TextSubstitution(text="640")
    )
    camera_elevation_launch_arg = DeclareLaunchArgument(
        "camera_elevation", default_value=TextSubstitution(text="0.171")
    )

    return LaunchDescription(
        [
            debug_enabled_launch_arg,
            camera_fov_launch_arg,
            camera_width_launch_arg,
            camera_elevation_launch_arg,
            Node(
                package="ml_robotics_project",
                executable="yolo_node",
                name="yolo_node",
                parameters=[
                    {
                        "debug_enabled": LaunchConfiguration("debug_enabled"),
                    }
                ],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="ml_robotics_project",
                executable="pos_finder_node",
                name="pos_finder_node",
                parameters=[
                    {
                        "debug_enabled": LaunchConfiguration("debug_enabled"),
                        "camera_fov": LaunchConfiguration("camera_fov"),
                        "camera_width": LaunchConfiguration("camera_width"),
                        "camera_elevation": LaunchConfiguration("camera_elevation"),
                    }
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
