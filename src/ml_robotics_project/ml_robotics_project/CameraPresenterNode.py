import threading

import rclpy
from rclpy.qos import QoSPresetProfiles

from ml_robotics_project.INode import INode
from ml_robotics_project.UI.ICameraViewAdapter import ICameraViewAdapter
from ml_robotics_interfaces.msg import BoundingBoxImage


class CameraPresenterNode(INode):
    """Node that presents camera feed to a camera view."""

    def __init__(self, node_name: str, camera_view_adapter: ICameraViewAdapter) -> None:
        super().__init__(node_name)
        self._camera_view = camera_view_adapter

    def _init_member_variables(self) -> None:
        pass

    def _init_ros_parameters(self) -> None:
        pass

    def _init_publishers(self) -> None:
        pass

    def _init_subscribers(self) -> None:
        self.bbox_image_sub = self.create_subscription(
            BoundingBoxImage,
            "bbox_image_data",
            self.bbox_image_sub_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _init_timers(self) -> None:
        pass

    def bbox_image_sub_callback(self, msg) -> None:
        self._debug("[bbox_image_sub] Received msg")
        self._camera_view.update_frame(msg)


def main(args=None) -> None:
    # TODO: Use dependency injection to inject the CameraView & CameraViewAdapter into the CameraPresenterNode instead of imports
    from ml_robotics_project.UI.CameraViewAdapter import CameraViewAdapter
    from ml_robotics_project.UI.CameraView import CameraView

    rclpy.init(args=args)

    # Build the "stack" of dependencies
    camera_view = CameraView()
    camera_view_adapter = CameraViewAdapter(camera_view)
    node = CameraPresenterNode("camera_presenter_node", camera_view_adapter)

    # Start the Qt event loop on a separate thread. "Daemon" means it terminates on shutdown.
    qt_thread = threading.Thread(target=run_qt_app, args=(), daemon=True)
    qt_thread.start()

    # Start the ROS event loop
    rclpy.spin(node)
    node.destroy_node()  # Destroy the node explicitly (optional)
    rclpy.shutdown()


def run_qt_app(args) -> None:
    # TODO: Find how to run Qt app more elegantly (and possibly from a single instance across all UI nodes)
    from PyQt5.QtWidgets import QApplication

    app = QApplication(*args)
    app.exec_()


if __name__ == "__main__":
    main()
