from typing import Optional

from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool

from .api import Api

class MagnetApi(Api):
    def __init__(self, node: Node):
        
        # TODO: qos_policy (Copied from autositter repo, might not fit this project)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.grab_status_pub = node.create_publisher(  # TODO change topic name
            Bool,
            # payload system subscribe to /drone_{i}/grab_status, for i from 0 to 3
            "grab_status",
            qos_profile
        )

    @property
    def is_loaded(self) -> bool:
        return self.__is_loaded
    
    # TODO
    # listen topic to update __is_loaded (not from camera)
    def activate_magnet(self) -> None:
        # TODO change msg type (custom)
        grab_status_msg = Bool()
        grab_status_msg.data = True
        self.grab_status_pub.publish(grab_status_msg)

    def deactivate_magnet(self) -> None:
        # TODO change msg type (custom)
        grab_status_msg = Bool()
        grab_status_msg.data = False
        self.grab_status_pub.publish(grab_status_msg)