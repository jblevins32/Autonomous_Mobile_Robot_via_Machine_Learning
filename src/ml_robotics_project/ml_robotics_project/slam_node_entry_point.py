"""This module implements the entry point for the SLAM node.

When the command 'ros2 run ml_robotics_project slam_node' is executed, this script is run.
"""

from pathlib import Path

import rclpy
from ml_robotics_project.ml_algorithms.Slam import Slam
from ml_robotics_project.algorithm_handlers.SlamHandler import SlamHandler
from ml_robotics_project.ros_nodes.SlamNode import SlamNode

import logging
logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler())
logging.basicConfig(filename="slam_node_entry_point.py", level=logging.INFO)

def main(args=None) -> None:
    rclpy.init(args=args)

    logger.info("Initializing SLAM algorithm...")
    slam = Slam()
    logger.info("Slam algorithm initialized.")
    
    logger.info("Initializing SlamHandler...")
    slam_handler = SlamHandler(slam)
    logger.info("SlamHandler initialized.")

    logger.info("Initializing SlamNode...")
    node = SlamNode("slam_node", slam_handler)
    logger.info("SlamNode initialized.")

    logger.info("Spinning node...")
    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
