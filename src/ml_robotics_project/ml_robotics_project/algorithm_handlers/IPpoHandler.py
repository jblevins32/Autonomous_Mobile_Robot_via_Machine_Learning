import abc

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


class IPpoHandler(metaclass=abc.ABCMeta):
    """Interface for PPO handlers."""

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "get_cmd_vel") and callable(subclass.get_cmd_vel)
        ) or NotImplemented

    @abc.abstractmethod
    def get_cmd_vel(self, odom_msg: Odometry, trajectory_msg: Path) -> Twist:
        """Gets the desired command velocity from the odometry and trajectory data.

        Parameters
        ----------
        odom_msg : Odometry
            ROS2 Odometry message.

        trajectory_msg : Path
            ROS2 Path message.

        Returns
        -------
        Twist
            ROS2 Twist message.
        """
        raise NotImplementedError
