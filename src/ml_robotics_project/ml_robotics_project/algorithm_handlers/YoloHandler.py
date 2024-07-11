import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from ml_robotics_interfaces.msg import BoundingBox

from ml_robotics_project.algorithm_handlers.IYoloHandler import IYoloHandler
from ml_robotics_project.ml_algorithms.Yolo import Yolo


class YoloHandler(IYoloHandler):
    """Handler for YOLO algorithm. This class accounts for data preprocessing and postprocessing"""

    def __init__(self, model_path: str) -> None:
        self._yolo = Yolo(model_path)
        self._bridge = CvBridge()

    def get_objective_coords(self, image_msg: Image) -> tuple[bool, Point]:
        # Preprocess data for YOLO
        success, image_data = self._preprocess_image(image_msg)
        if not success:
            return False, BoundingBox()

        # Reference to the YOLO algorithm
        results = self._yolo.get_objective_coords(image_data)

        # Postprocess data for ROS
        success, bounding_box = self._postprocess_results(results)
        if not success:
            return False, BoundingBox()

        return True, bounding_box

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

    def _postprocess_results(self, results):
        x1, y1, x2, y2 = results.boxes.xyxy
        return BoundingBox(
            top_left=Point(x=x1, y=y1, z=0.0), bottom_right=Point(x=x2, y=y2, z=0.0)
        )
