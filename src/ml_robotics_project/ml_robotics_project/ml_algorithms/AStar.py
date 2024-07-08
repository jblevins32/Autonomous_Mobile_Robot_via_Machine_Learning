import numpy as np

from ml_robotics_project.ml_algorithms.IAStar import IAStar


class AStar(IAStar):
    """AStar algorithm class."""

    def __init__(self) -> None:
        pass

    def get_trajectory(
        self, grid_map_data: np.ndarray, objective_coords: np.ndarray
    ) -> np.ndarray:
        # TODO Implement this method

        # AStar algorithm implementation

        return np.arange(3)
