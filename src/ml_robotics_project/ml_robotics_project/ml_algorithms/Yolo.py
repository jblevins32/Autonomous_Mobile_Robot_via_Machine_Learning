import numpy as np
from ultralytics import YOLO

from ml_robotics_project.ml_algorithms.IYolo import IYolo


class Yolo(IYolo):
    """YOLO algorithm class."""

    def __init__(self, model_path: str) -> None:
        self.model = YOLO(model_path)

    def get_objective_coords(self, image_data: np.ndarray):
        results = self.model(image_data, stream=True)
        return results[0].cpu().numpy()
