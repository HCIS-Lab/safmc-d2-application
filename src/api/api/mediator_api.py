from typing import List, Optional

from rclpy.node import Node
from std_msgs.msg import Bool

from agent_msgs.msg import (AgentStatus, DropZoneInfo, ObstacleArray,
                            SupplyZoneInfo)
from common.coordinate import Coordinate
from common.qos import cmd_qos_profile

from .api import Api


class MediatorApi(Api):
    _is_ok_to_takeoff = False
    _is_ok_to_drop = False
    _supply_zone: List[Optional[Coordinate]] = [None, None, None, None]
    _drop_zone: Optional[Coordinate] = None
    _obstacle_array: List[Coordinate] = []  # list

    def __init__(self, node: Node):
        prefix = "mediator/"

        # Subscriptions
        node.create_subscription(
            Bool, prefix + "cmd_takeoff", self._set_is_ok_to_takeoff, cmd_qos_profile
        )

        node.create_subscription(
            Bool, prefix + "cmd_drop", self._set_is_ok_to_drop, cmd_qos_profile
        )

        node.create_subscription(
            SupplyZoneInfo,
            prefix + "supply_zone",
            self._set_supply_zone,
            cmd_qos_profile,
        )

        node.create_subscription(
            DropZoneInfo, prefix + "drop_zone", self._set_drop_zone, cmd_qos_profile
        )

        node.create_subscription(
            ObstacleArray,
            prefix + "obstacle_array",
            self._set_obstacle_array,
            cmd_qos_profile,
        )

        # Publishers

        self.status_pub = node.create_publisher(
            AgentStatus, prefix + "status", cmd_qos_profile
        )

        self.online_pub = node.create_publisher(
            Bool, prefix + "online", cmd_qos_profile
        )
        self.drop_request_pub = node.create_publisher(
            Bool, prefix + "drop_request", cmd_qos_profile
        )
        self.drop_ack_pub = node.create_publisher(
            Bool, prefix + "drop_ack", cmd_qos_profile
        )

    def online(self):
        msg = Bool()
        msg.data = True
        self.online_pub.publish(msg)

    def wait_to_drop(self):
        msg = Bool()
        msg.data = True
        self.drop_request_pub.publish(msg)

    def send_drop_ack(self):
        msg = Bool()
        msg.data = True
        self.drop_ack_pub.publish(msg)

    # TODO status 重寫
    def send_status(self, state_name: str, local_position: Coordinate):
        if state_name is None or local_position is None:
            return
        agent_status_msg = AgentStatus()
        agent_status_msg.point.x = float(local_position.x)
        agent_status_msg.point.y = float(local_position.y)
        agent_status_msg.point.z = float(local_position.z)
        self.status_pub.publish(agent_status_msg)

    @property
    def is_ok_to_takeoff(self):
        return self._is_ok_to_takeoff

    @property
    def is_ok_to_drop(self):
        return self._is_ok_to_drop

    def reset_states(self):
        """
        重置 is_ok_to_takeoff, is_ok_to_drop
        """
        self._is_ok_to_takeoff = False
        self._is_ok_to_drop = False

    @property
    def supply_zone(self):
        return self._supply_zone

    @property
    def drop_zone(self):
        return self._drop_zone

    @property
    def obstacle_array(self) -> List[Coordinate]:
        return self._obstacle_array

    def _set_is_ok_to_takeoff(self, msg: Bool):
        self._is_ok_to_takeoff = msg.data

    def _set_is_ok_to_drop(self, msg: Bool):
        self._is_ok_to_drop = msg.data

    def _set_supply_zone(self, msg: SupplyZoneInfo):
        self._supply_zone[0] = Coordinate.from_point(msg.point_1)
        self._supply_zone[1] = Coordinate.from_point(msg.point_2)
        self._supply_zone[2] = Coordinate.from_point(msg.point_3)
        self._supply_zone[3] = Coordinate.from_point(msg.point_4)

    def _set_drop_zone(self, msg: DropZoneInfo):
        self._drop_zone = Coordinate.from_point(msg.point)

    def _set_obstacle_array(self, msg: ObstacleArray):
        self._obstacle_array = [Coordinate.from_point(point) for point in msg.points]
