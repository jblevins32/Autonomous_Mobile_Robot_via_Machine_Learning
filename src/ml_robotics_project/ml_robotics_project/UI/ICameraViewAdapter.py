import abc

from ml_robotics_interfaces.msg import BoundingBoxImage


class ICameraViewAdapter(metaclass=abc.ABCMeta):
    """Interface for a camera view adapter that updates the camera view with the given bounding box image."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "update_frame") and callable(subclass.update_frame)
        ) or NotImplemented

    @abc.abstractmethod
    def update_frame(self, bbox_image_msg: BoundingBoxImage) -> None:
        """Updates the camera view with the given bounding box image.

        Parameters
        ----------
        update_frame : BoundingBoxImage
            ROS2 BoundingBoxImage message.
        """
        raise NotImplementedError
