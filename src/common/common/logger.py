from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node


class Logger:
    _instance = None
    _logger: RcutilsLogger

    def __new__(cls, node: Node = None):
        if cls._instance is None:
            if node is None:
                raise ValueError(
                    "Logger is not initialized yet! Please initialize it with a ROS 2 Node."
                )
            cls._instance = super().__new__(cls)
            cls._instance._logger = node.get_logger()
        return cls._instance

    @classmethod
    def _get_logger(cls):
        if cls._instance is None:
            raise ValueError(
                "Logger is not initialized yet! Please initialize it with a ROS 2 Node."
            )
        return cls._instance._logger

    @classmethod
    def debug(cls, msg, *args):
        cls._get_logger().debug(msg, *args)

    @classmethod
    def info(cls, msg, *args):
        cls._get_logger().info(msg, *args)

    @classmethod
    def warning(cls, msg, *args):
        cls._get_logger().warning(msg, *args)

    @classmethod
    def error(cls, msg, *args):
        cls._get_logger().error(msg, *args)

    @classmethod
    def fatal(cls, msg, *args):
        cls._get_logger().fatal(msg, *args)
