from sensor_msgs.msg import Image

from ml_robotics_project.UI.ICameraView import ICameraView
from ml_robotics_project.UI.ICameraViewAdapter import ICameraViewAdapter
from ml_robotics_interfaces.msg import BoundingBoxImage

from cv_bridge import CvBridge
import numpy as np
import cv2


class CameraViewAdapter(ICameraViewAdapter):
    """A wrapper for the ICameraView class that converts incoming data into data that the ICameraView can use."""

    def __init__(self, camera_view: ICameraView) -> None:
        self._camera_view = camera_view
        self._bridge = CvBridge()
        self._camera_view.show()

    def update_frame(self, bbox_image_msg: BoundingBoxImage) -> None:
        image_data = self._preprocess_data(bbox_image_msg)
        self._camera_view.update_frame(image_data)

    def _preprocess_data(self, bbox_image_msg: BoundingBoxImage) -> np.ndarray:
        image_data = self._preprocess_image(bbox_image_msg.image)
        if self._bounding_box_valid(bbox_image_msg.bounding_box):
            top_left, bottom_right = self._preprocess_bounding_box(
                bbox_image_msg.bounding_box
            )
            image_data = self._draw_bounding_box(image_data, top_left, bottom_right)
        return image_data

    def _preprocess_image(self, image_msg: Image) -> np.ndarray:
        cv_image = self._bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image_data = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        return image_data

    def _bounding_box_valid(self, bounding_box_msg: BoundingBox) -> bool:
        return bounding_box_msg is not None and not (
            bounding_box_msg.top_left.x == bounding_box_msg.bottom_right.x
            and bounding_box_msg.top_left.y == bounding_box_msg.bottom_right.y
        )

    def _preprocess_bounding_box(
        self, bounding_box_msg: BoundingBox
    ) -> tuple[tuple[int, int], tuple[int, int]]:
        top_left = (bounding_box_msg.top_left.x, bounding_box_msg.top_left.y)
        bottom_right = (
            bounding_box_msg.bottom_right.x,
            bounding_box_msg.bottom_right.y,
        )
        return top_left, bottom_right

    def _draw_bounding_box(
        self,
        image_data: np.ndarray,
        top_left: tuple[int, int],
        bottom_right: tuple[int, int],
    ) -> np.ndarray:
        image_data = cv2.rectangle(image_data, top_left, bottom_right, (0, 255, 0), 2)
        return image_data
