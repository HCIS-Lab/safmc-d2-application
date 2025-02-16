from typing import List, Optional

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, UInt32

from agent_msgs.msg import (AgentStatus, DropZoneInfo, ObstacleArray,
                            SupplyZoneInfo)
from common.coordinate import Coordinate

from .api import Api


class MediatorApi(Api):
    def __init__(self, node: Node, drone_id: int):

        self.__drone_id = drone_id

        self.__clock: Clock = node.get_clock()

        # Initial Values
        self.__is_ok_to_arm = False  # 是否可以 arming (當同組的所有 drone 都上線後由 mediator 下指令)
        self.__is_ok_to_takeoff = False
        self.__is_ok_to_drop = False
        self.__received_disarm_signal = False

        self.__supply_zone: List[Optional[Coordinate]] = [None, None, None, None]
        self.__drop_zone: Optional[Coordinate] = None

        self.__obstacle_array: List[Coordinate] = []  # list

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        node.create_subscription(
            SupplyZoneInfo,
            f"/agent_{self.__drone_id}/supply_zone",
            self.__set_supply_zone,
            qos_profile
        )

        node.create_subscription(
            DropZoneInfo,
            f"/agent_{self.__drone_id}/drop_zone",
            self.__set_drop_zone,
            qos_profile
        )

        node.create_subscription(
            Bool,
            f"/agent_{self.__drone_id}/arm",
            self.__set_is_ok_to_arm,
            qos_profile
        )

        node.create_subscription(
            Bool,
            f"/agent_{self.drone_id}/ext_disarm",
            self.__disarm,
            qos_profile
        )

        node.create_subscription(
            Bool,
            f"/agent_{self.__drone_id}/takeoff",
            self.__set_is_ok_to_takeoff,
            qos_profile
        )

        node.create_subscription(
            Bool,
            f"/agent_{self.__drone_id}/drop",
            self.__set_is_ok_to_drop,
            qos_profile
        )

        node.create_subscription(
            ObstacleArray,
            f"/agent_{drone_id+1}/obstacle_array",
            self.__set_obstacle_array,
            qos_profile
        )

        # Publishers

        self.status_pub = node.create_publisher(
            AgentStatus,
            '/mediator/status',
            qos_profile
        )

        self.online_pub = node.create_publisher(
            UInt32,
            '/mediator/online',
            qos_profile
        )

        self.arm_ack_pub = node.create_publisher(
            UInt32,
            '/mediator/arm_ack',
            qos_profile
        )

        self.wait_pub = node.create_publisher(
            UInt32,
            '/mediator/wait',
            qos_profile
        )

        self.drop_ack_pub = node.create_publisher(
            UInt32,
            '/mediator/drop_ack',
            qos_profile
        )

    def __get_drone_id_msg(self) -> UInt32:
        uint32_msg = UInt32()
        uint32_msg.data = self.__drone_id
        return uint32_msg

    def online(self):
        self.online_pub.publish(self.__get_drone_id_msg())

    def arm_ack(self):
        self.arm_ack_pub.publish(self.__get_drone_id_msg())

    def wait_to_drop(self):
        self.wait_pub.publish(self.__get_drone_id_msg())

    def send_drop_ack(self):
        self.drop_ack_pub.publish(self.__get_drone_id_msg())

    # TODO status 重寫
    def send_status(self, state_name: str, local_position: Coordinate):
        if state_name is None or local_position is None:
            return
        agent_status_msg = AgentStatus()
        agent_status_msg.drone_id = self.__drone_id
        agent_status_msg.local_position.x = float(local_position.x)
        agent_status_msg.local_position.y = float(local_position.y)
        agent_status_msg.local_position.z = float(local_position.z)
        self.status_pub.publish(agent_status_msg)

    @property
    def is_ok_to_arm(self):
        return self.__is_ok_to_arm

    @property
    def is_ok_to_takeoff(self):
        return self.__is_ok_to_takeoff

    @property
    def is_ready_to_drop(self):
        return self.__is_ready_to_drop

    @property
    def supply_zone(self):
        return self.__supply_zone

    @property
    def drop_zone(self):
        return self.__drop_zone4
    
    def reset_arm_status(self):
        self.__is_ready_to_arm = False
        self.__is_ready_to_takeoff = False
        self.__is_ready_to_drop = False

    @property
    def obstacle_array(self) -> List[Coordinate]:
        return self.__obstacle_array

    def __set_is_ok_to_arm(self, msg: Bool):
        self.__is_ok_to_arm = msg.data

    def __set_is_ok_to_takeoff(self, msg: Bool):
        self.__is_ok_to_takeoff = msg.data

    def __set_is_ok_to_drop(self, msg: Bool):
        self.__is_ok_to_drop = msg.data

    def __disarm(self, msg: Bool):
        self.__received_disarm_signal = msg.data

    def __set_supply_zone(self, msg: SupplyZoneInfo):
        self.__supply_zone[0] = Coordinate.from_point(msg.point_1)
        self.__supply_zone[1] = Coordinate.from_point(msg.point_2)
        self.__supply_zone[2] = Coordinate.from_point(msg.point_3)
        self.__supply_zone[3] = Coordinate.from_point(msg.point_4)

    def __set_drop_zone(self, msg: DropZoneInfo):
        self.__drop_zone = Coordinate.from_point(msg.point)

    def __set_obstacle_array(self, msg: ObstacleArray):
        self.__obstacle_array = [Coordinate.from_point(point) for point in msg.points]
