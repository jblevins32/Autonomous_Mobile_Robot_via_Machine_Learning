"""
Interface for PPO class. This interface is used to ensure the PPO class implements the required methods.
"""
import abc
import numpy as np


class IPpo(metaclass=abc.ABCMeta):
    """Interface for PPO."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "get_cmd_vel") and callable(subclass.get_cmd_vel)
        ) or NotImplemented

    @abc.abstractmethod
    def get_cmd_vel(self, odom_data: np.ndarray, trajectory_data: np.ndarray) -> np.ndarray:
        """Gets the desired command velocity from the odometry and trajectory data.

        Parameters
        ----------
        odom_data : ndarray
            Odometry array.

        trajectory_data : ndarray
            Path array.

        Returns
        -------
        ndarray
            Twist array.
        """
        raise NotImplementedError
