import rclpy
from rclpy.qos import QoSPresetProfiles
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point

from ml_robotics_project.INode import INode
from ml_robotics_project.algorithm_handlers.AStarHandler import AStarHandler


class AStarNode(INode):
    """Node that computes trajectory given a grid map and objective coordinates."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info("AStarNode started")

    def _init_member_variables(self) -> None:
        self._a_star_handler = AStarHandler()
        self._curr_grid_map_msg = None
        self._curr_objective_coords_msg = None

    def _init_ros_parameters(self) -> None:
        pass

    def _init_publishers(self) -> None:
        self.trajectory_pub = self.create_publisher(
            Path,
            "trajectory",
            10,
        )

    def _init_subscribers(self) -> None:
        self.grid_map_sub = self.create_subscription(
            OccupancyGrid,
            "grid_map",
            self.grid_map_sub_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.objective_coords_sub = self.create_subscription(
            Point,
            "objective_coords",
            self.objective_coords_sub_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _init_timers(self) -> None:
        self.trajectory_pub_timer = self.create_timer(
            timer_period_sec=0.1,
            callback=self.trajectory_pub_timer_callback,
        )

    def grid_map_sub_callback(self, msg) -> None:
        self._debug(f"[grid_map_sub] Received: {msg}")
        self._curr_grid_map_msg = msg

    def objective_coords_sub_callback(self, msg) -> None:
        self._debug(f"[objective_coords_sub] Received: {msg}")
        self._curr_objective_coords_msg = msg

    def trajectory_pub_timer_callback(self) -> None:
        if self._curr_grid_map_msg is None:
            return

        if self._curr_objective_coords_msg is None:
            # TODO handle this case
            self._curr_objective_coords_msg = Point(x=0, y=0, z=0)

        msg = self._a_star_handler.get_trajectory(
            self._curr_grid_map_msg, self._curr_objective_coords_msg
        )
        self._debug(f"[trajectory_pub] Publishing: {msg}")
        self.trajectory_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarNode("a_star_node")
    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
