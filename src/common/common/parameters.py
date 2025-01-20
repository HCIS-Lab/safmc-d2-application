from typing import Any

from rclpy.node import Node


def get_parameter(node: Node, name: str, default_value: Any) -> Any:
    """
    Utility to get a parameter value with a fallback to a default value.
    """

    if not node.has_parameter(name):
        param = node.declare_parameter(name, default_value)
    else:
        param = node.get_parameter(name).value

    return param.value
