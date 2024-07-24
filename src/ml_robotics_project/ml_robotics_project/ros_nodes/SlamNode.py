import rclpy
import rclpy.logging
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

from ml_robotics_project.INode import INode
from ml_robotics_project.algorithm_handlers.SlamHandler import SlamHandler


class SlamNode(INode):
    """Node that performs SLAM using lidar and odometry data."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info("SlamNode started")

    def _init_member_variables(self) -> None:
        self._slam_handler = SlamHandler()
        self._curr_odom_msg = None
        self._curr_lidar_msg = None

    def _init_ros_parameters(self) -> None:
        pass

    def _init_publishers(self) -> None:
        self.grid_map_pub = self.create_publisher(
            OccupancyGrid,
            "grid_map",
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _init_subscribers(self) -> None:
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_sub_callback,
            QoSPresetProfiles.SENSOR_DATA.value,  # QoS profile can be changed
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.lidar_sub_callback,
            10,
        )

    def _init_timers(self) -> None:
        self.grid_map_pub_timer = self.create_timer(
            0.1,
            self.grid_map_pub_timer_callback,
        )

    def odom_sub_callback(self, msg) -> None:
        self._debug(f"[odom_sub] Received: {msg}")
        self._curr_odom_msg = msg

    def lidar_sub_callback(self, msg) -> None:
        self._debug(f"[lidar_sub] Received: {msg}")
        self._curr_lidar_msg = msg

    def grid_map_pub_timer_callback(self) -> None:
        if self._curr_odom_msg is None or self._curr_lidar_msg is None:
            return
        
        msg = self._slam_handler.get_grid_map(self._curr_odom_msg, self._curr_lidar_msg)
        self._debug(f"[grid_map_pub] Publishing: {msg}")
        self.grid_map_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamNode("slam_node")
    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
