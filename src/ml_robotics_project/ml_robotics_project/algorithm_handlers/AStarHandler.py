import numpy as np

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point

from ml_robotics_project.algorithm_handlers.IAStarHandler import IAStarHandler
from ml_robotics_project.ml_algorithms.AStar import AStar


class AStarHandler(IAStarHandler):
    """Handler for AStar algorithm. This class accounts for data preprocessing and postprocessing"""

    def __init__(self) -> None:
        self._a_star = AStar()

    def get_trajectory(
        self, grid_map_msg: OccupancyGrid, objective_coords_msg: Point
    ) -> Path:
        # TODO Implement this method

        # Preprocess data for AStar

        # Reference to the AStar algorithm
        result = self._a_star.get_trajectory(np.arange(3), np.arange(3))

        # Postprocess data for ROS

        return Path()
