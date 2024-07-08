import numpy as np

from ml_robotics_project.ml_algorithms.IYolo import IYolo


class Yolo(IYolo):
    """YOLO algorithm class."""

    def __init__(self) -> None:
        pass

    def get_objective_coords(self, image_data: np.ndarray) -> tuple[bool, np.ndarray]:
        # TODO Implement this method

        # YOLO algorithm implementation

        return False, np.arange(3)
