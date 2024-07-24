import rclpy
from rclpy.qos import QoSPresetProfiles
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist

from ml_robotics_project.INode import INode
from ml_robotics_project.algorithm_handlers.PpoHandler import PpoHandler


class PpoNode(INode):
    """Node that computes desired robot velocity given odometry and trajectory data."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info("PpoNode started")

    def _init_member_variables(self) -> None:
        self._ppo_handler = PpoHandler()
        self._curr_odom_msg = None
        self._curr_trajectory_msg = None

    def _init_ros_parameters(self) -> None:
        pass

    def _init_publishers(self) -> None:
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "cmd_vel",
            10,
        )

    def _init_subscribers(self) -> None:
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_sub_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.trajectory_sub = self.create_subscription(
            Path,
            "trajectory",
            self.trajectory_sub_callback,
            10,
        )

    def _init_timers(self) -> None:
        self.cmd_vel_pub_timer = self.create_timer(
            0.1,
            self.cmd_vel_pub_timer_callback,
        )

    def odom_sub_callback(self, msg) -> None:
        self._debug(f"[odom_sub] Received: {msg}")
        self._curr_odom_msg = msg

    def trajectory_sub_callback(self, msg) -> None:
        self._debug(f"[trajectory_sub] Received: {msg}")
        self._curr_trajectory_msg = msg

    def cmd_vel_pub_timer_callback(self) -> None:
        if self._curr_odom_msg is None or self._curr_trajectory_msg is None:
            return

        msg = self._ppo_handler.get_cmd_vel(
            self._curr_odom_msg, self._curr_trajectory_msg
        )
        self._debug(f"[cmd_vel_pub] Publishing: {msg}")
        self.cmd_vel_pub.publish(msg)
