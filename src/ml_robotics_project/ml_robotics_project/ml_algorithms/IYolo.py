"""
This module implements the interface for the YOLO class. This interface is used to ensure the YOLO class implements the required methods.
"""
import abc
import numpy as np


class IYolo(metaclass=abc.ABCMeta):
    """Interface for YOLO."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "update")
            and callable(subclass.update)
            and hasattr(subclass, "get_results")
            and callable(subclass.get_results)
        ) or NotImplemented

    @abc.abstractmethod
    def update(self, image_data: np.ndarray) -> None:
        """Updates the YOLO algorithm with the new image data.

        Parameters
        ----------
        image_data : ndarray
            Image data array.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_results(self):
        """Gets the results from the YOLO algorithm.

        Returns
        -------
        Results
            Ultralytics YOLO results.
        """
        raise NotImplementedError
