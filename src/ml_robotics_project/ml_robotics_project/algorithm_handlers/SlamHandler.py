import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

from ml_robotics_project.algorithm_handlers.ISlamHandler import ISlamHandler
from ml_robotics_project.ml_algorithms.Slam import Slam


class SlamHandler(ISlamHandler):
    """Handler for SLAM algorithm. This class accounts for data preprocessing and postprocessing"""

    def __init__(self, slam: Slam) -> None:
        self._slam = slam

    def get_grid_map(self, odom_msg: Odometry, lidar_msg: LaserScan) -> OccupancyGrid:
        # TODO Implement this method

        # Preprocess data for SLAM

        # Reference to the SLAM algorithm
        result = self._slam.get_grid_map(np.arange(3), np.arange(3))

        # Postprocess data for ROS

        return OccupancyGrid()
