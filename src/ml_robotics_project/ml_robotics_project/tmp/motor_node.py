import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from nav_msgs.msg import Odometry

# from ftm_project import ftm_functions as F

class Motor_Node(Node):

    DEBUG = False

    def __init__(self):
        super().__init__("motor_node")  # Initialize & name the node
        self.init_member_variables()
        self.init_ros_parameters()
        self.init_publishers()
        self.init_subscribers()
        self.init_timers()

    def init_member_variables(self):
        """All member variables are declared and initialized here."""
        self.bool_var = False

    def init_publishers(self):
        """All publishers are declared and initialized here."""
        self.publisher = self.create_publisher(
            msg_type=Ftm,  # Custom interface can go here
            topic="topic_name",
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,  # QoS profile can be changed
        )

    def init_subscribers(self):
        """All subscriptions are declared and initialized here."""
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="topic_name",
            callback=self.subscriber_1_callback,
            qos_profile=QoSPresetProfiles.SENSOR_DATA.value,  # QoS profile can be changed
        )
        self._ftm_pair_sub = self.create_subscription(
            msg_type=FtmPair,  # Custom interface can go here
            topic="topic_name_2",
            callback=self.subscriber_2_callback,
            qos_profile=10,
        )

    def init_timers(self):
        """All timers are declared and initialized here."""
        self.timer = self.create_timer(
            timer_period_sec=0.1,
            callback=self.timer_callback,
        )

    def subscriber_1_callback(self, msg):
        # Do stuff here. You can use regular python functions from different modules here.
        pass

    def subscriber_2_callback(self, msg):
        pass

    def timer_callback(self):
        # Timer callbacks can be used to publish messages at a fixed rate
        msg = Ftm()
        self.publisher.publish(msg)

    def init_ros_parameters(self):
        """All ROS parameters are declared and initialized here (if any)."""
        self.declare_parameter(
            name="param_name",
            value=0.0,
        )

    @property
    def param_name(self):
        return (
            self.get_parameter("param_name")
            .get_parameter_value()
            .double_value  # This can be double, integer, bool, etc.
        )

    def debug(self, msg):
        """Convenience method for sending debug logs.
        This method will only send logs if the DEBUG flag is set to True.

        Parameters
        ----------
        msg : str
            message to be logged
        """
        if self.DEBUG:
            self.get_logger().debug(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Motor_Node()
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
