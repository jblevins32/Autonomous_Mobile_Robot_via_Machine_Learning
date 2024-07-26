"""This module contains the PosFinderNode class.

This node uses 2D lidar and bounding box data to find the 3D position of an object."""
import rclpy
import rclpy.logging
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import numpy as np

from ml_robotics_interfaces.msg import BoundingBox
from ml_robotics_project.ros_nodes.INode import INode


class PosFinderNode(INode):
    """Node that uses lidar and bounding box data to find the position of an object."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info("PosFinderNode started")

    def _init_member_variables(self) -> None:
        self._curr_bbox_msg = None
        self._curr_lidar_msg = None

    def _init_ros_parameters(self) -> None:
        self.declare_parameter(
            "camera_fov",
            3.089,  # Field of view of pi camera, in radians
        )
        self.declare_parameter(
            "camera_width",
            640,  # Width of pi camera image, in pixels
        )
        self.declare_parameter(
            "camera_elevation",
            value=0.171,  # Elevation of camera from ground, in meters
        )

    @property
    def camera_fov(self):
        return self.get_parameter("camera_fov").get_parameter_value().double_value

    @property
    def camera_width(self):
        return self.get_parameter("camera_width").get_parameter_value().integer_value

    @property
    def camera_elevation(self):
        return self.get_parameter("camera_elevation").get_parameter_value().double_value

    def _init_publishers(self) -> None:
        self.objective_pos_pub = self.create_publisher(
            Point,
            "objective_rel_pos",
            10,
        )

    def _init_subscribers(self) -> None:
        self.bbox_sub = self.create_subscription(
            BoundingBox,
            "bounding_box",
            self.bbox_sub_callback,
            10,
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.lidar_sub_callback,
            10,
        )

    def _init_timers(self) -> None:
        self.objective_pos_pub_timer = self.create_timer(
            0.1,
            self.objective_pos_pub_timer_callback,
        )

    def bbox_sub_callback(self, msg) -> None:
        self._debug(f"[bbox_sub] Received: {msg}")
        self._curr_bbox_msg = msg

    def lidar_sub_callback(self, msg) -> None:
        self._debug(f"[lidar_sub] Received: {msg}")
        self._curr_lidar_msg = msg

    def objective_pos_pub_timer_callback(self) -> None:
        # Both messages must have been received this frame
        if self._curr_bbox_msg is None or self._curr_lidar_msg is None:
            return

        # Calculate the position of the object
        x1 = self._curr_bbox_msg.top_left[0]
        x2 = self._curr_bbox_msg.bottom_right[0]

        x1_index = self._get_lidar_data_index(
            x1, self.camera_width, self.camera_fov, self._curr_lidar_msg.angle_increment
        )
        x2_index = self._get_lidar_data_index(
            x2, self.camera_width, self.camera_fov, self._curr_lidar_msg.angle_increment
        )

        x1_angle = self._get_x_angle(x1, self.camera_width, self.camera_fov)
        x2_angle = self._get_x_angle(x2, self.camera_width, self.camera_fov)

        x1_index = x1_angle // self._curr_lidar_msg.angle_increment
        x2_index = x2_angle // self._curr_lidar_msg.angle_increment

        x1_range = self._curr_lidar_msg.ranges[x1_index]
        x2_range = self._curr_lidar_msg.ranges[x2_index]

        # TODO check that the 2 points are part of the same object here (using the clusters from DBSCAN). If they aren't, return

        x1_relative_coords = x1_range * np.array([np.cos(x1_angle), np.sin(x1_angle)])
        x2_relative_coords = x2_range * np.array([np.cos(x2_angle), np.sin(x2_angle)])

        x_avg_relative_coords = (x1_relative_coords + x2_relative_coords) / 2
        msg = Point(
            x=x_avg_relative_coords[0],
            y=x_avg_relative_coords[1],
            z=self.camera_elevation,
        )
        self._debug(f"[objective_pos_pub] Publishing: {msg}")
        self.objective_pos_pub.publish(msg)

        # Reset values for next frame
        self._curr_bbox_msg = None
        self._curr_lidar_msg = None

    def _get_lidar_data_index(self, x, width, fov, angle_increment):
        x_dist_per_radian = width / fov
        if x <= width / 2:
            index = ((width / 2 - x) / x_dist_per_radian) // angle_increment
        else:
            index = -((x - width / 2) / x_dist_per_radian) // angle_increment
        return index

    def _get_x_angle(self, x, width, fov):
        x_dist_per_radian = width / fov
        return (width / 2 - x) / x_dist_per_radian
