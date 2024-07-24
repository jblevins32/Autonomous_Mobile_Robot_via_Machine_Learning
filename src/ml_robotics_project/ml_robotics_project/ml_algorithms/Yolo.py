"""
This module implements the YOLO algorithm. It uses the Ultralytics YOLO library on the YOLOv8 model, which is robust and open-source.
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
        self._results = self._model(image_data)

    def get_results(self):
        if self._results is None:
            return None
        return self._results[0].cpu().numpy()
