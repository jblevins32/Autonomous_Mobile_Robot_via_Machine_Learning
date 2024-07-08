import numpy as np

from ml_robotics_project.ml_algorithms.IPpo import IPpo


class Ppo(IPpo):
    """PPO algorithm class."""

    def __init__(self) -> None:
        pass

    def get_cmd_vel(
        self, odom_data: np.ndarray, trajectory_data: np.ndarray
    ) -> np.ndarray:
        # TODO Implement this method

        # PPO algorithm implementation

        return np.arange(3)
