import rclpy
from ml_robotics_project.INode import INode


class NodeTmp(INode):

    def _init_member_variables(self) -> None:
        pass

    def _init_ros_parameters(self) -> None:
        pass

    def _init_publishers(self) -> None:
        pass

    def _init_subscribers(self) -> None:
        pass

    def _init_timers(self) -> None:
        pass


def main(args=None):
    rclpy.init(args=args)

    node = NodeTmp("node_tmp")
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
