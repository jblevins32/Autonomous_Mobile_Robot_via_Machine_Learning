"""
This module defines the interface for the SLAM class. This interface is used to ensure the SLAM class implements the required methods.
"""

import abc
import numpy as np


class ISlam(metaclass=abc.ABCMeta):
    """Interface for SLAM."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "get_grid_map") and callable(subclass.get_grid_map)
        ) or NotImplemented

    @abc.abstractmethod
    def get_grid_map(self, odom_data: np.ndarray, lidar_data: np.ndarray) -> np.ndarray:
        """Gets the occupancy grid map from the odometry and lidar data.

        Parameters
        ----------
        odom_data : ndarray
            Odometry array.

        lidar_data : ndarray
            Laser scan array.

        Returns
        -------
        ndarray
            Occupancy grid map array.
        """
        raise NotImplementedError
