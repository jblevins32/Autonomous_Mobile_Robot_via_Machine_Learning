import numpy as np

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist

from ml_robotics_project.algorithm_handlers.IPpoHandler import IPpoHandler
from ml_robotics_project.ml_algorithms.Ppo import Ppo


class PpoHandler(IPpoHandler):
    """Handler for PPO algorithm. This class accounts for data preprocessing and postprocessing"""

    def __init__(self, ppo: Ppo) -> None:
        self._ppo = ppo

    def get_cmd_vel(self, odom_msg: Odometry, trajectory_msg: Path) -> Twist:
        # TODO Implement this method

        # Preprocess data for PPO

        # Reference to the PPO algorithm
        result = self._ppo.get_cmd_vel(np.arange(3), np.arange(3))

        # Postprocess data for ROS

        return Twist()
