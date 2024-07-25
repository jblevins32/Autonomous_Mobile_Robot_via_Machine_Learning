from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ml_robotics_project.ros_nodes.INode import INode


class CameraNode(INode):
    """Node that captures images from a camera and publishes them."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info("CameraNode started")

    def _init_member_variables(self) -> None:
        self._bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # create a video capture object for camera 0

    def _init_ros_parameters(self) -> None:
        pass

    def _init_publishers(self) -> None:
        self.image_pub = self.create_publisher(Image, "camera/image_raw", 10)

    def _init_subscribers(self) -> None:
        pass

    def _init_timers(self) -> None:
        self.image_pub_timer = self.create_timer(
            0.1,
            self.image_pub_timer_callback,
        )

    def image_pub_timer_callback(self) -> None:
        ret, frame = self.cap.read()
        if ret:  # Check if an image has been successfully captured
            msg = self._bridge.cv2_to_imgmsg(
                frame, encoding="bgr8"
            )  # convert cv2 to ROS image
            self.image_pub.publish(msg)  # publish the message
            self.get_logger().info(
                "Publishing Image"
            )  # Log that I am publishing the image

    def destroy(self) -> None:
        self.cap.release()
        super().destroy_node()
