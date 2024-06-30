from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    collect_time_launch_arg = DeclareLaunchArgument(
        "collect_time", default_value=TextSubstitution(text="2.5")
    )
    processing_delay_launch_arg = DeclareLaunchArgument(
        "processing_delay", default_value=TextSubstitution(text="0.15")
    )
    ftm_offset_launch_arg = DeclareLaunchArgument(
        "ftm_offset", default_value=TextSubstitution(text="0.0")
    )
    debug_mode_launch_arg = DeclareLaunchArgument(
        "debug_mode", default_value=TextSubstitution(text="False")
    )

    return LaunchDescription(
        [
            collect_time_launch_arg,
            processing_delay_launch_arg,
            ftm_offset_launch_arg,
            debug_mode_launch_arg,
            Node(
                package="ftm_project",
                executable="ftm_server",
                name="ftm_server",  # Node name
                # Set parameters here
                parameters=[
                    {
                        "collect_time": LaunchConfiguration("collect_time"),
                        "processing_delay": LaunchConfiguration("processing_delay"),
                        "ftm_offset": LaunchConfiguration("ftm_offset"),
                        "debug_mode": LaunchConfiguration("debug_mode"),
                    }
                ],
            ),
            Node(
                package="ftm_project",
                executable="multilateration",
                name="multilateration",  # Node name
            ),
        ]
    )
