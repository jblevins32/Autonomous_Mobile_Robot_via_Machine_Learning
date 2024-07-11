import sys
import asyncio

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

    # def main(args=None) -> None:
    #     # TODO: Use dependency injection to inject the CameraView & CameraViewAdapter into the CameraPresenterNode instead of imports
    #     from ml_robotics_project.UI.CameraViewAdapter import CameraViewAdapter
    #     from ml_robotics_project.UI.CameraView import CameraView

    #     # TODO: Find how to run Qt app more elegantly (and possibly from a single instance across all UI nodes)
    #     from PyQt5.QtWidgets import QApplication

    #     qt_app = QApplication(sys.argv)
    #     rclpy.init(args=args)

    #     # Build the "stack" of dependencies
    #     camera_view = CameraView()
    #     camera_view_adapter = CameraViewAdapter(camera_view)
    #     node = CameraPresenterNode("camera_presenter_node", camera_view_adapter)

    #     # Start the Qt event loop on a separate thread. "Daemon" means it terminates on shutdown.
    #     # qt_thread = threading.Thread(target=run_qt_app, args=(), daemon=True)
    #     # qt_thread.start()

    #     # Start the ROS event loop
    #     rclpy.spin(node)
    #     node.destroy_node()  # Destroy the node explicitly (optional)
    #     rclpy.shutdown()

    # def run_qt_app(app) -> None:
    #     app.exec_()


# TODO: Use dependency injection to inject the CameraView & CameraViewAdapter into the CameraPresenterNode instead of imports
from ml_robotics_project.UI.CameraViewAdapter import CameraViewAdapter
from ml_robotics_project.UI.CameraView import CameraView

# TODO: Find how to run Qt app more elegantly (and possibly from a single instance across all UI nodes)
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer


# async def run_qt_app(app):
#     loop = asyncio.get_running_loop()
#     return await loop.run_in_executor(None, app.exec_)


# async def spin_ros_node(node):
#     rclpy.spin(node)
#     QApplication.quit()


# async def main_async(args=None) -> None:
#     rclpy.init(args=args)
#     app = QApplication(sys.argv)

#     camera_view = CameraView()
#     camera_view_adapter = CameraViewAdapter(camera_view)
#     node = CameraPresenterNode("camera_presenter_node", camera_view_adapter)

#     qt_task = asyncio.create_task(run_qt_app(app))
#     ros_task = asyncio.create_task(spin_ros_node(node))

#     # await asyncio.gather(qt_task, ros_task)

#     # Wait for the ROS task to complete. This implies that once the ROS node
#     # is done, we proceed to shutdown the Qt application if it's still running.
#     await ros_task
#     # If the Qt app is still running, wait for it to finish. This step might be
#     # instantaneous if the Qt app was already instructed to quit in spin_ros_node.
#     await qt_task

#     node.destroy_node()
#     rclpy.shutdown()


# def main():
#     asyncio.run(main_async())


# if __name__ == "__main__":
#     main()

#     # loop = asyncio.get_event_loop()
#     # loop.run_until_complete(main())
#     # loop.close()


def check_ros_events(node):
    """
    Check for ROS events and process them without blocking.
    """
    rclpy.spin_once(node, timeout_sec=0)


async def main_async(args=None) -> None:
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # Initialize your ROS node and UI components here
    camera_view = CameraView()
    camera_view_adapter = CameraViewAdapter(camera_view)
    node = CameraPresenterNode("camera_presenter_node", camera_view_adapter)

    # Set up a QTimer to periodically check ROS events very frequently
    timer = QTimer()
    timer.timeout.connect(lambda: check_ros_events(node))
    timer.start(10)  # Check ROS events every 10ms

    # Process any initial ROS events before starting the Qt event loop
    check_ros_events(node)

    # Execute the Qt application
    exit_code = app.exec_()

    # Clean up ROS node
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
