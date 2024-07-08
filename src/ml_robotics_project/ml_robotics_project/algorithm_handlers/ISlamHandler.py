import abc

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid


class ISlamHandler(metaclass=abc.ABCMeta):
    """Interface for SLAM handlers."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "get_grid_map") and callable(subclass.get_grid_map)
        ) or NotImplemented

    @abc.abstractmethod
    def get_grid_map(self, odom_msg: Odometry, lidar_msg: LaserScan) -> OccupancyGrid:
        """Gets the occupancy grid map from the odometry and lidar data.

        Parameters
        ----------
        odom_msg : Odometry
            ROS2 Odometry message.

        lidar_msg : LaserScan
            ROS2 LaserScan message.

        Returns
        -------
        OccupancyGrid
            ROS2 OccupancyGrid message.
        """
        raise NotImplementedError
