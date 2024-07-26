from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    debug_enabled_launch_arg = DeclareLaunchArgument(
        "debug_enabled", default_value=TextSubstitution(text="False")
    )

    return LaunchDescription(
        [
            debug_enabled_launch_arg,
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
                executable="slam_node",
                name="slam_node",
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
                executable="ppo_node",
                name="ppo_node",
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
                executable="a_star_node",
                name="a_star_node",
                parameters=[
                    {
                        "debug_enabled": LaunchConfiguration("debug_enabled"),
                    }
                ],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
