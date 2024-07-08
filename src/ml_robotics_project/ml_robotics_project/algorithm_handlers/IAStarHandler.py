import abc

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Path


class IAStarHandler(metaclass=abc.ABCMeta):
    """Interface for AStar handlers."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "get_trajectory") and callable(subclass.get_trajectory)
        ) or NotImplemented

    @abc.abstractmethod
    def get_trajectory(
        self, grid_map_msg: OccupancyGrid, objective_coords_msg: Point
    ) -> Path:
        """Gets the robot's trajectory given the grid map and objective coordinates.

        Parameters
        ----------
        grid_map_msg : OccupancyGrid
            ROS2 OccupancyGrid message.

        objective_coords_msg : Point
            ROS2 Point message.

        Returns
        -------
        Path
            ROS2 Path message.
        """
        raise NotImplementedError
