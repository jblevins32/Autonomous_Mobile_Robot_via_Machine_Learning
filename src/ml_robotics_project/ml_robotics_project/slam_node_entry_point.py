"""This module implements the entry point for the SLAM node.

When the command 'ros2 run ml_robotics_project slam_node' is executed, this script is run.
"""

from pathlib import Path

import rclpy
from ml_robotics_project.ml_algorithms.Yolo import Slam
from ml_robotics_project.algorithm_handlers.PpoHandler import SlamHandler
from ml_robotics_project.ros_nodes.SlamNode import SlamNode

import logging
logging.basicConfig(level=logging.INFO)

def main(args=None) -> None:
    rclpy.init(args=args)
    logger = logging.getLogger("slam").addHandler(logging.StreamHandler())

    model_path = Path.cwd() / "ml_models" / "yolo_models"/ "best100ep.pt"
    ppo = Ppo()
    yolo_handler = YoloHandler(yolo)
    node = YoloNode("yolo_node", yolo_handler)

    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PpoNode("ppo_node")
    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
