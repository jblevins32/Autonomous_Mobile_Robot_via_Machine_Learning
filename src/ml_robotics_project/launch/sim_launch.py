import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution,
)

from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
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
    
    # Launch files to include
    # TODO: Include using get_package_share_directory WITHOUT errors
    turtlebot_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    # get_package_share_directory("turtlebot3_gazebo"),  # Package name
                    os.getcwd(),  # Workspace directory
                    "src",
                    "turtlebot3_simulations",  # Package name
                    "turtlebot3_gazebo",
                    "launch",  # Launch folder
                ),
                "/turtlebot3_world.launch.py",  # Launch file name
            ]
        ),
    )
    rel_pos_finder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    os.getcwd(),  # Workspace directory
                    "src",
                    "ml_robotics_project",  # Package name
                    "launch",  # Launch folder
                ),
                "/rel_pos_finder_launch.py",  # Launch file name
            ]
        ),
        launch_arguments={
            "debug_enabled": debug_enabled_launch_arg.default_value,
            "camera_fov": camera_fov_launch_arg.default_value,
            "camera_width": camera_width_launch_arg.default_value,
            "camera_elevation": camera_elevation_launch_arg.default_value,
        }.items(),
    )

    return LaunchDescription(
        [
            debug_enabled_launch_arg,
            camera_fov_launch_arg,
            camera_width_launch_arg,
            camera_elevation_launch_arg,
            turtlebot_sim_launch,
            rel_pos_finder_launch,
        ]
    )
