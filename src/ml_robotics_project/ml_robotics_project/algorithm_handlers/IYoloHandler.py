"""This module implements the interface for YOLO handlers.

This interface is used to define the methods that YOLO handlers must implement.
This is makes the system easy to modify and extend, as different YOLO handlers can be created by implementing this interface.
"""

import abc

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ml_robotics_interfaces.msg import BoundingBox


class IYoloHandler(metaclass=abc.ABCMeta):
    """Interface for YOLO handlers."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "update")
            and callable(subclass.update)
            and hasattr(subclass, "get_objective_bounding_box")
            and callable(subclass.get_objective_bounding_box)
            and hasattr(subclass, "get_bounding_box_image")
            and callable(subclass.get_bounding_box_image)
        ) or NotImplemented

    @abc.abstractmethod
    def update(self, image_msg: Image) -> None:
        """Updates the YOLO algorithm with a new image.

        Parameters
        ----------
        image_msg : Image
            The image data to update the YOLO algorithm with.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_objective_bounding_box(self) -> tuple[bool, BoundingBox]:
        """Gets the bounding box of the objective object.

        Returns
        -------
        tuple[bool, BoundingBox]
            A tuple containing a boolean indicating success and the bounding box data.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def get_bounding_box_image(self) -> tuple[bool, Image]:
        """Gets the bounding box image.

        Returns
        -------
        tuple[bool, Image]
            A tuple containing a boolean indicating success and the image containing the bounding box.
        """
        raise NotImplementedError
