import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CompressedImage
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
        success, image_data = self._preprocess_image(image_msg)
        if success:
            self._yolo.update(image_data)

    def get_objective_bounding_box(self) -> tuple[bool, BoundingBox]:
        # Get results from YOLO
        results = self._yolo.get_results()
        if results is None:
            return False, BoundingBox()

        # Postprocess data for ROS
        bounding_box = self._postprocess_bounding_box(results)
        return True, bounding_box

    def get_bounding_box_image(self) -> tuple[bool, Image]:
        # Get results from YOLO
        results = self._yolo.get_results()
        if results is None:
            return False, Image()

        # Get bounding box coordinates
        top_left = (results.boxes.xyxy[0][0], results.boxes.xyxy[0][1])
        bottom_right = (results.boxes.xyxy[0][2], results.boxes.xyxy[0][3])

        # Draw bounding box
        bounding_box_image = self._draw_bounding_box(
            results.orig_img, top_left, bottom_right
        )

        # Convert image to ROS Image
        image_msg = self._bridge.cv2_to_imgmsg(bounding_box_image, "rgb8")
        return True, image_msg

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
        x1, y1, x2, y2 = results.boxes.xyxy[0]
        return BoundingBox(
            top_left=Point(x=float(x1), y=float(y1), z=0.0), bottom_right=Point(x=float(x2), y=float(y2), z=0.0)
        )
