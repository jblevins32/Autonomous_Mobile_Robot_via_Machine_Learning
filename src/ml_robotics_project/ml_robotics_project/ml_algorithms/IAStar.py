import abc

import numpy as np


class IAStar(metaclass=abc.ABCMeta):
    """Interface for AStar."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "get_trajectory") and callable(subclass.get_trajectory)
        ) or NotImplemented

    @abc.abstractmethod
    def get_trajectory(
        self, grid_map_data: np.ndarray, objective_coords: np.ndarray
    ) -> np.ndarray:
        """Gets the robot's trajectory given the grid map and objective coordinates.

        Parameters
        ----------
        grid_map_data : ndarray
            Occupancy grid map array.

        objective_coords : ndarray
            Point array.

        Returns
        -------
        ndarray
            Path array.
        """
        raise NotImplementedError
