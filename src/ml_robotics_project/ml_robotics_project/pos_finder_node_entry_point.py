"""This module implements the entry point for the pos_finder node.

When the command 'ros2 run ml_robotics_project pos_finder_node' is executed, this script is run.
"""

from ml_robotics_project.ros_nodes.PosFinderNode import PosFinderNode
import rclpy

import logging
logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler())
logging.basicConfig(filename="pos_finder_node_entry_point.py", level=logging.INFO)

def main(args=None) -> None:
    rclpy.init(args=args)

    logger.info("Initializing PosFinderNode...")
    node = PosFinderNode("pos_finder_node")
    logger.info("PosFinderNode initialized.")

    logger.info("Spinning node...")
    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


if __name__ == "__main__":
    main()