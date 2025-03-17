from rclpy.node import Node
from std_msgs.msg import Bool
from safmc_msgs.msg import DropZoneInfo, SupplyZoneInfo
from std_msgs.msg import Int32
from common.qos import cmd_qos_profile

from .api import Api


class MediatorApi(Api):

    _is_ok_to_takeoff = False
    _is_ok_to_drop = False

    _supply_zone_marker_id = -1
    _drop_zone_marker_id = -1

    _supply_zone_code = ""
    _drop_zone_code = ""

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

        # Publishers
        self.status_pub = node.create_publisher(
            Int32, prefix + "status", cmd_qos_profile
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

    @property
    def is_ok_to_takeoff(self):
        return self._is_ok_to_takeoff

    @property
    def is_ok_to_drop(self):
        return self._is_ok_to_drop

    @property
    def supply_zone_code(self):
        return self._drop_zone_code

    @property
    def supply_zone_marker_id(self):
        return self._supply_zone_marker_id

    @property
    def drop_zone_code(self):
        return self._drop_zone_code

    @property
    def drop_zone_marker_id(self):
        return self._drop_zone_marker_id

    def _set_is_ok_to_takeoff(self, msg: Bool):
        self._is_ok_to_takeoff = msg.data

    def _set_is_ok_to_drop(self, msg: Bool):
        self._is_ok_to_drop = msg.data

    def _set_supply_zone(self, msg: SupplyZoneInfo):
        self._supply_zone_code = msg.supply_zone_code
        self._supply_zone_marker_id = msg.aruco_marker_id

    def _set_drop_zone(self, msg: DropZoneInfo):
        self._drop_zone_code = msg.drop_zone_code
        self._drop_zone_marker_id = msg.aruco_marker_id
