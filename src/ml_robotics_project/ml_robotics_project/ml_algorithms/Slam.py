"""
This module implements the SLAM algorithm.
"""

import numpy as np

from ml_robotics_project.ml_algorithms.ISlam import ISlam


class Slam(ISlam):
    """SLAM algorithm class."""

    def __init__(self) -> None:
        pass

    def get_grid_map(self, odom_data: np.ndarray, lidar_data: np.ndarray) -> np.ndarray:
        # TODO Implement this method

        # SLAM algorithm implementation

        return np.arange(3)
