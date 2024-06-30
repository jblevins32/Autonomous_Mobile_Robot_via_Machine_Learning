from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    ftm_period_launch_arg = DeclareLaunchArgument(
        "ftm_period", default_value=TextSubstitution(text="0.5")
    )
    ftm_proc_delay_launch_arg = DeclareLaunchArgument(
        "ftm_proc_delay", default_value=TextSubstitution(text="0.15")
    )
    ftm_offset_launch_arg = DeclareLaunchArgument(
        "ftm_offset", default_value=TextSubstitution(text="0.0")
    )

    return LaunchDescription(
        [
            ftm_period_launch_arg,
            ftm_proc_delay_launch_arg,
            ftm_offset_launch_arg,
            Node(
                package="ftm_project",
                executable="ftm_publisher",
                name="ftm_publisher",  # Node name
                # Print output to the console
                output="screen",
                emulate_tty=True,
                # Set parameters here
                parameters=[
                    {
                        "ftm_period": LaunchConfiguration("ftm_period"),
                        "ftm_proc_delay": LaunchConfiguration("ftm_proc_delay"),
                        "ftm_offset": LaunchConfiguration("ftm_offset"),
                    }
                ],
            ),
        ]
    )
