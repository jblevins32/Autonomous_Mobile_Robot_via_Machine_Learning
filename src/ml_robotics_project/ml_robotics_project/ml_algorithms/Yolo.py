"""
This module implements the YOLO algorithm. It uses the Ultralytics YOLO library on the YOLOv8 model, which is fast, robust, and open-source.
"""

import numpy as np
from ultralytics import YOLO

from ml_robotics_project.ml_algorithms.IYolo import IYolo


class Yolo(IYolo):
    """YOLO algorithm class."""

    def __init__(self, model_path: str) -> None:
        self._model = YOLO(model_path)
        self._results = None

    def update(self, image_data: np.ndarray) -> None:
        """Runs the YOLO model on the input image and saves the results"""
        self._results = self._model(image_data)

    def get_results(self):
        """Returns the results of the YOLO model"""
        if self._results is None:
            return None
        return self._results[0].cpu().numpy()
