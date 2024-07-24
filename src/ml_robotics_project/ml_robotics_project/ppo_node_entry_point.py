"""This module implements the entry point for the YOLO node.

When the command 'ros2 run ml_robotics_project ppo_node' is executed, this script is run.
"""

import logging
from pathlib import Path

import rclpy
from ml_robotics_project.ml_algorithms.Yolo import Ppo
from ml_robotics_project.algorithm_handlers.PpoHandler import PpoHandler
from ml_robotics_project.ros_nodes.PpoNode import PpoNode


def main(args=None) -> None:
    rclpy.init(args=args)
    logger = logging.getLogger().addHandler(logging.StreamHandler())

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
