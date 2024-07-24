    """This module implements the entry point for the YOLO node.

    When the command 'ros2 run ml_robotics_project yolo_node' is executed, this script is run.
    """

from pathlib import Path

import rclpy
from ml_robotics_project.ml_algorithms.Yolo import Yolo
from ml_robotics_project.algorithm_handlers.YoloHandler import YoloHandler
from ml_robotics_project.ros_nodes.YoloNode import YoloNode


def main(args=None) -> None:
    rclpy.init(args=args)

    model_path = Path.cwd() / "ml_models" / "yolo_models"/ "best100ep.pt"
    yolo = Yolo(model_path)
    yolo_handler = YoloHandler(yolo)
    node = YoloNode("yolo_node", yolo_handler)

    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
