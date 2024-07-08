import abc

import numpy as np


class IYolo(metaclass=abc.ABCMeta):
    """Interface for YOLO."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "get_objective_coords")
            and callable(subclass.get_objective_coords)
        ) or NotImplemented

    @abc.abstractmethod
    def get_objective_coords(self, image_data: np.ndarray) -> tuple[bool, np.ndarray]:
        """Gets the objective coordinates from the image message.

        Parameters
        ----------
        image_data : ndarray
            Image data array.

        Returns
        -------
        tuple[bool, ndarray]
            A tuple of boolean and an array representing a point in space. The boolean indicates if the image contains the objective.
            The point contains the objective coordinates.
        """
        raise NotImplementedError
