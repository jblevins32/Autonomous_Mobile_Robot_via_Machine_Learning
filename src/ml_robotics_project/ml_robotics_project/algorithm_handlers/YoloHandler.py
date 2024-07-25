"""This module implements the YOLO algorithm handler.

This class accounts for data preprocessing and postprocessing.
That is, it converts data from ROS messages to data that can be used by the YOLO algorithm.
Once the YOLO algorithm is run, it converts the results back to ROS messages.
"""

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from ml_robotics_interfaces.msg import BoundingBox

from ml_robotics_project.algorithm_handlers.IYoloHandler import IYoloHandler
from ml_robotics_project.ml_algorithms.IYolo import IYolo


class YoloHandler(IYoloHandler):
    """Handler for YOLO algorithm. This class accounts for data preprocessing and postprocessing"""

    def __init__(self, yolo: IYolo) -> None:
        self._yolo = yolo
        self._bridge = CvBridge()

    def update(self, image_msg: Image) -> None:
        """Updates the YOLO algorithm with a new image"""
        success, image_data = self._preprocess_image(image_msg)
        if success:
            self._yolo.update(image_data)

    def get_objective_bounding_box(self) -> tuple[bool, BoundingBox]:
        """Returns the bounding box of the objective in the image

        Returns
        -------
        bool
            Whether the bounding box was successfully obtained
        BoundingBox
            The bounding box of the objective. This is a custom ROS message.
        """
        # Get results from YOLO
        results = self._yolo.get_results()
        if not self._valid_results(results):
            return False, BoundingBox()

        # Postprocess data for ROS
        bounding_box = self._postprocess_bounding_box(results)
        return True, bounding_box

    def get_bounding_box_image(self) -> tuple[bool, Image]:
        """Returns an image with the bounding box of the objective

        Returns
        -------
        bool
            Whether the image was successfully obtained
        Image
            ROS image with the bounding box
        """
        # Get results from YOLO
        results = self._yolo.get_results()
        if not self._valid_results(results):
            return False, Image()

        # Get bounding box coordinates
        top_left = (int(results.boxes.xyxy[0][0]), int(results.boxes.xyxy[0][1]))
        bottom_right = (int(results.boxes.xyxy[0][2]), int(results.boxes.xyxy[0][3]))

        # Draw bounding box
        bounding_box_image = self._draw_bounding_box(
            results.orig_img, top_left, bottom_right
        )
        

        # Convert image to ROS Image
        image_msg = self._bridge.cv2_to_imgmsg(bounding_box_image, "rgb8")
        return True, image_msg

    def _valid_results(self, results):
        return (
            results is not None
            and results.boxes is not None
            and results.boxes.xyxy is not None
            and len(results.boxes.xyxy) != 0
        )

    def _draw_bounding_box(
        self,
        image_data: np.ndarray,
        top_left: tuple[int, int],
        bottom_right: tuple[int, int],
    ) -> np.ndarray:
        image_data = cv2.rectangle(image_data, top_left, bottom_right, (0, 255, 0), 2)
        return image_data

    def _preprocess_image(self, image_msg: Image) -> tuple[bool, np.ndarray]:
        # Convert into OpenCV image
        try:
            cv_image = self._bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            print(f"Error converting ROS Image to OpenCV: {e}")
            return False, np.array([])

        # Convert into RGB numpy array
        image_data = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_data = np.asarray(image_data)
        return True, image_data

    def _postprocess_bounding_box(self, results):
        x1 = results.boxes.xyxy[0][0]
        y1 = results.boxes.xyxy[0][1]
        x2 = results.boxes.xyxy[0][2]
        y2 = results.boxes.xyxy[0][3]
        return BoundingBox(
            top_left=Point(x=float(x1), y=float(y1), z=0.0),
            bottom_right=Point(x=float(x2), y=float(y2), z=0.0),
        )
