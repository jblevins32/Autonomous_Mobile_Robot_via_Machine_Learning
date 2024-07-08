import rclpy
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from ml_robotics_project.INode import INode
from ml_robotics_project.algorithm_handlers.YoloHandler import YoloHandler


class YoloNode(INode):
    """Node that computes objective coordinates given an image."""

    def _init_member_variables(self) -> None:
        self._yolo_handler = YoloHandler()
        self._curr_image_msg = None

    def _init_ros_parameters(self) -> None:
        pass

    def _init_publishers(self) -> None:
        self.objective_coords_pub = self.create_publisher(
            Point,
            "objective_coords",
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _init_subscribers(self) -> None:
        self.image_sub = self.create_subscription(
            Image,
            "image_data",
            self.image_sub_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _init_timers(self) -> None:
        self.objective_coords_pub_timer = self.create_timer(
            0.1,
            self.objective_coords_pub_timer_callback,
        )

    def image_sub_callback(self, msg) -> None:
        self._debug(f"[image_sub] Received: {msg}")
        self._curr_image_msg = msg

    def objective_coords_pub_timer_callback(self) -> None:
        if self._curr_image_msg is None:
            return

        success, msg = self._yolo_handler.get_objective_coords(self._curr_image_msg)
        if success:
            self._debug(f"[objective_coords_pub] Publishing: {msg}")
            self.objective_coords_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloNode("yolo_node")
    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
