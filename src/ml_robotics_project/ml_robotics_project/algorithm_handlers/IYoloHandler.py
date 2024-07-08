import abc

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point


class IYoloHandler(metaclass=abc.ABCMeta):
    """Interface for YOLO handlers."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "get_objective_coords")
            and callable(subclass.get_objective_coords)
        ) or NotImplemented

    @abc.abstractmethod
    def get_objective_coords(self, image_msg: Image) -> tuple[bool, Point]:
        """Gets the objective coordinates from the image message.

        Parameters
        ----------
        image_msg : Image
            ROS2 Image message.

        Returns
        -------
        tuple[bool, Point]
            A tuple of boolean and ROS2 Point message. The boolean indicates if the image contains the objective.
            The ROS2 Point message contains the objective coordinates.
        """
        raise NotImplementedError
