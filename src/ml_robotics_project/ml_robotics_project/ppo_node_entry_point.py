"""This module implements the entry point for the YOLO node.

When the command 'ros2 run ml_robotics_project ppo_node' is executed, this script is run.
"""

import logging
from pathlib import Path

import rclpy
from ml_robotics_project.ml_algorithms.Ppo import Ppo
from ml_robotics_project.algorithm_handlers.PpoHandler import PpoHandler
from ml_robotics_project.ros_nodes.PpoNode import PpoNode


import logging
logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler())
logging.basicConfig(filename="slam_node_entry_point.py", level=logging.INFO)

def main(args=None) -> None:
    rclpy.init(args=args)

    logger.info("Initializing PPO algorithm...")
    ppo = Ppo()
    logger.info("Ppo algorithm initialized.")
    
    logger.info("Initializing PpoHandler...")
    ppo_handler = PpoHandler(ppo)
    logger.info("PpoHandler initialized.")

    logger.info("Initializing PpoNode...")
    node = PpoNode("ppo_node", ppo_handler)
    logger.info("PpoNode initialized.")

    logger.info("Spinning node...")
    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
