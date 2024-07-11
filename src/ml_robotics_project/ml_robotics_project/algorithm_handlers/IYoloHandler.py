import abc

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ml_robotics_interfaces.msg import BoundingBox


class IYoloHandler(metaclass=abc.ABCMeta):
    """Interface for YOLO handlers."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "get_objective_coords")
            and callable(subclass.get_objective_coords)
        ) or NotImplemented

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
