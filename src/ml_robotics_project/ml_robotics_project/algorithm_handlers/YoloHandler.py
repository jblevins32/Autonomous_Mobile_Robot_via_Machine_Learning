import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from ml_robotics_project.algorithm_handlers.IYoloHandler import IYoloHandler
from ml_robotics_project.ml_algorithms.Yolo import Yolo


class YoloHandler(IYoloHandler):
    """Handler for YOLO algorithm. This class accounts for data preprocessing and postprocessing"""

    def __init__(self) -> None:
        self._yolo = Yolo()

    def get_objective_coords(self, image_msg: Image) -> tuple[bool, Point]:
        # TODO Implement this method

        # Preprocess data for YOLO

        # Reference to the YOLO algorithm
        result = self._yolo.get_objective_coords(np.arange(3))

        # Postprocess data for ROS

        return False, Point()
