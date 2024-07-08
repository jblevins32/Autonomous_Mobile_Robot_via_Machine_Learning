from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


def construct_pose_stamped(
    x_pos, y_pos, z_pos, x_orient, y_orient, z_orient, w_orient
) -> PoseStamped:
    """Constructs a ROS2 PoseStamped message."""
    return PoseStamped(
        pose=Pose(
            position=Point(
                x=x_pos,
                y=y_pos,
                z=z_pos,
            ),
            orientation=Quaternion(
                x=x_orient,
                y=y_orient,
                z=z_orient,
                w=w_orient,
            ),
        ),
    )
