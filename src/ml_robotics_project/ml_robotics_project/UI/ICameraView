import abc
import numpy as np


class ICameraView:
    """Interface for a camera view. It enforces the implementation of the contained methods."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "update_frame") and callable(subclass.update_frame)
        ) or NotImplemented

    @abc.abstractmethod
    def update_frame(self, rgb_image: np.ndarray) -> None:
        """Updates the camera output frame with the provided RGB image."""
        raise NotImplementedError
