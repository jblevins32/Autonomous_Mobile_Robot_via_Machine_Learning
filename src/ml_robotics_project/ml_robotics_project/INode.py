import abc
from rclpy.node import Node


class INode(Node, metaclass=abc.ABCMeta):

    def __init__(self, node_name: str, **kwargs) -> None:
        """Initializes member variables, ros parameters, publishers, subscribers, and timers."""
        super().__init__(node_name, **kwargs)

        self.declare_parameter(
            "debug_enabled",
            False,
        )

        self._init_member_variables()
        self._init_ros_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()

        self._debug(f'Node "{node_name}" initialized.')

    @classmethod
    def __subclasshook__(cls, subclass):
        return (
            hasattr(subclass, "_init_member_variables")
            and callable(subclass._init_member_variables)
            and hasattr(subclass, "_init_ros_parameters")
            and callable(subclass._init_ros_parameters)
            and hasattr(subclass, "_init_publishers")
            and callable(subclass._init_publishers)
            and hasattr(subclass, "_init_subscribers")
            and callable(subclass._init_subscribers)
            and hasattr(subclass, "_init_timers")
            and callable(subclass._init_timers)
        ) or NotImplemented

    @abc.abstractmethod
    def _init_member_variables(self) -> None:
        """All member variables are declared and initialized here."""
        raise NotImplementedError

    @abc.abstractmethod
    def _init_ros_parameters(self) -> None:
        """All ROS parameters are declared and initialized here (if any)."""
        raise NotImplementedError

    @abc.abstractmethod
    def _init_publishers(self) -> None:
        """All publishers are declared and initialized here."""
        raise NotImplementedError

    @abc.abstractmethod
    def _init_subscribers(self) -> None:
        """All subscriptions are declared and initialized here."""
        raise NotImplementedError

    @abc.abstractmethod
    def _init_timers(self) -> None:
        """All timers are declared and initialized here."""
        raise NotImplementedError

    def _debug(self, msg: str) -> None:
        if self.debug_enabled:
            self.get_logger().info(msg)

    @property
    def debug_enabled(self):
        return self.get_parameter("debug_enabled").get_parameter_value().bool_value
