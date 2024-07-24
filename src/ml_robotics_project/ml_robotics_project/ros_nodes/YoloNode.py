"""
This module implements the YOLO node.
This node is responsible for computing objective coordinates given an image.
It receives a ROS2 Image message as input and sends a custom-made BoundingBox message as output.
It also optionally publishes new ROS2 Images with superimposed bounding boxes for visualization purposes.
"""

import rclpy
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

from ml_robotics_interfaces.msg import BoundingBox
from ml_robotics_project.INode import INode
from ml_robotics_project.algorithm_handlers.IYoloHandler import IYoloHandler


class YoloNode(INode):
    """Node that computes objective coordinates given an image."""

    def __init__(self, node_name: str, yolo_handler: IYoloHandler) -> None:
        super().__init__(node_name)
        self._yolo_handler = yolo_handler
        self._curr_image_msg = None

    def _init_member_variables(self) -> None:
        pass

    def _init_ros_parameters(self) -> None:
        pass

    def _init_publishers(self) -> None:
        self.objective_bbox_pub = self.create_publisher(
            BoundingBox,
            "bounding_box",
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.bbox_image_pub = self.create_publisher(
            Image,
            "bounding_box_image",
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
        self.objective_bbox_pub_timer = self.create_timer(
            0.1,
            self.objective_bbox_pub_timer_callback,
        )

    def image_sub_callback(self, msg) -> None:
        self._debug(f"[image_sub] Received: {msg}")
        self._curr_image_msg = msg

    def objective_bbox_pub_timer_callback(self) -> None:
        if self._curr_image_msg is None:
            return

        self._yolo_handler.update(self._curr_image_msg)

        # Publish bounding box data
        success, bbox_msg = self._yolo_handler.get_objective_bounding_box()
        if success:
            self._debug(f"[objective_bbox_pub] Publishing: {bbox_msg}")
            self.objective_bbox_pub.publish(bbox_msg)

        # Publish new image with bounding box (for visualization purposes)
        success, bbox_image_msg = self._yolo_handler.get_bounding_box_image()
        if success:
            self._debug(f"[bbox_image_pub] Publishing bounding box image")
            self.bbox_image_pub.publish(bbox_image_msg)
