from typing import List, Optional

from rclpy.node import Node
from std_msgs.msg import Bool

from agent_msgs.msg import DropZoneInfo, ObstacleArray, SupplyZoneInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Int32
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
            Int32, prefix + "status", cmd_qos_profile
        )

        self.agent_local_pos_pub = node.create_publisher(
            Point, prefix + "agent_local_pos", cmd_qos_profile
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
        msg = Bool(data=True)
        self.drop_ack_pub.publish(msg)

    def send_status(self, state_value: int):
        msg = Int32(data=state_value)
        self.status_pub.publish(msg)

    def send_agent_local_position(self, agent_local_position: Coordinate):
        msg = Point(
            x=agent_local_position.x, y=agent_local_position.y, z=agent_local_position.z
        )
        self.agent_local_pos_pub.publish(msg)

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
