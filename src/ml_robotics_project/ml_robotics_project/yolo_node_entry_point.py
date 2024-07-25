"""This module implements the entry point for the YOLO node.

When the command 'ros2 run ml_robotics_project yolo_node' is executed, this script is run.
"""

from pathlib import Path
import logging
logging.basicConfig(level=logging.INFO)

import rclpy
from ml_robotics_project.ml_algorithms.Yolo import Yolo
from ml_robotics_project.algorithm_handlers.YoloHandler import YoloHandler
from ml_robotics_project.ros_nodes.YoloNode import YoloNode

import logging
logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler())
logging.basicConfig(filename="slam_node_entry_point.py", level=logging.INFO)

def main(args=None) -> None:
    rclpy.init(args=args)

    logger.info("Initializing YOLO algorithm...")
    model_path = Path.cwd() / "ml_models" / "yolo_models"/ "best100ep.pt"
    yolo = Yolo(model_path)
    logger.info("Yolo algorithm initialized.")
    
    logger.info("Initializing YoloHandler...")
    yolo_handler = YoloHandler(yolo)
    logger.info("YoloHandler initialized.")

    logger.info("Initializing YoloNode...")
    node = YoloNode("yolo_node", yolo_handler)
    logger.info("YoloNode initialized.")

    logger.info("Spinning node...")
    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

