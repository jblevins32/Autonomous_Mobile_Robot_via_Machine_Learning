import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
import asyncio

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry


import numpy as np
from irobot_create_msgs.srv import ResetPose
from ml_robotics_interfaces.srv import GetFtm

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ml_robotics_interfaces.msg import Ftm, FtmPair
from ml_robotics_project.old import ftm_functions as F

# from ftm_project import ftm_functions as F


class NodeTemplate(Node):

    DEBUG = False

    def __init__(self):
        super().__init__("node_name")  # Initialize & name the node
        self.init_member_variables()
        self.init_ros_parameters()
        self.init_publishers()
        self.init_subscribers()
        self.init_timers()

    def init_member_variables(self):
        """All member variables are declared and initialized here."""
        self.bool_var = False

    def init_publishers(self):
        """All publishers are declared and initialized here."""
        self.publisher = self.create_publisher(
            msg_type=Ftm,  # Custom interface can go here
            topic="topic_name",
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,  # QoS profile can be changed
        )

    def init_subscribers(self):
        """All subscriptions are declared and initialized here."""
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="topic_name",
            callback=self.subscriber_1_callback,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,  # QoS profile can be changed
        )
        self._ftm_pair_sub = self.create_subscription(
            msg_type=FtmPair,  # Custom interface can go here
            topic="topic_name_2",
            callback=self.subscriber_2_callback,
            qos_profile=10,
        )

    def init_timers(self):
        """All timers are declared and initialized here."""
        self.timer = self.create_timer(
            timer_period_sec=0.1,
            callback=self.timer_callback,
        )

    def subscriber_1_callback(self, msg):
        # Do stuff here. You can use regular python functions from different modules here.
        pass

    def subscriber_2_callback(self, msg):
        pass

    def timer_callback(self):
        # Timer callbacks can be used to publish messages at a fixed rate
        msg = Ftm()
        self.publisher.publish(msg)

    def init_ros_parameters(self):
        """All ROS parameters are declared and initialized here (if any)."""
        self.declare_parameter(
            name="param_name",
            value=0.0,
        )

    @property
    def param_name(self):
        return (
            self.get_parameter("param_name")
            .get_parameter_value()
            .double_value  # This can be double, integer, bool, etc.
        )

    def debug(self, msg):
        """Convenience method for sending debug logs.
        This method will only send logs if the DEBUG flag is set to True.

        Parameters
        ----------
        msg : str
            message to be logged
        """
        if self.DEBUG:
            self.get_logger().debug(msg)


def main(args=None):
    rclpy.init(args=args)

    node = NodeTemplate()
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
