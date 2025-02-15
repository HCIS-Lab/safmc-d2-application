from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool, Empty, Int8

from agent_msgs.msg import AgentInfo, AgentStatus, DropZoneInfo, SupplyZoneInfo
from common.ned_coordinate import NEDCoordinate

from .api import Api


class MediatorApi(Api):
    def __init__(self, node: Node, drone_id: int):

        self.drone_id = drone_id

        self.__clock: Clock = node.get_clock()

        # Initial Values
        self.__is_ready_to_arm = False  # 是否可以 arming (當同組的所有 drone 都上線後由 mediator 下指令)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions

        self.__signal = False

        self.__supply_zone = [None, None]
        self.__drop_zone = None

        self.__is_ready_to_takeoff = False

        self.supply_zone_sub = node.create_subscription(
            SupplyZoneInfo,
            f"/agent_{self.drone_id}/supply_zone",
            self.__set_supply_zone,
            qos_profile
        )

        self.drop_zone_sub = node.create_subscription(
            DropZoneInfo,
            f"/agent_{self.drone_id}/drop_zone",
            self.__set_drop_zone,
            qos_profile
        )

        self.arm_sub = node.create_subscription(
            Bool,
            f"/agent_{self.drone_id}/arm",
            self.__arm,
            qos_profile
        )

        self.takeoff_sub = node.create_subscription(
            Bool,
            f"/agent_{self.drone_id}/takeoff",
            self.__takeoff,
            qos_profile
        )

        # Publishers

        self.online_pub = node.create_publisher(
            AgentInfo,
            '/mediator/online',
            qos_profile
        )

        self.arm_ack_pub = node.create_publisher(
            AgentInfo,
            '/mediator/arm_ack',
            qos_profile
        )

        self.status_pub = node.create_publisher(
            AgentStatus,
            '/mediator/status',
            qos_profile
        )

    # api functions

    def online(self):
        agent_info_msg = AgentInfo()
        agent_info_msg.timestamp = int(self.__clock.now().nanoseconds / 1000)
        agent_info_msg.drone_id = self.drone_id
        self.online_pub.publish(agent_info_msg)

    def arm_ack(self):
        agent_info_msg = AgentInfo()
        agent_info_msg.timestamp = int(self.__clock.now().nanoseconds / 1000)
        agent_info_msg.drone_id = self.drone_id
        self.arm_ack_pub.publish(agent_info_msg)

    def send_status(self, state_name: str, local_position: NEDCoordinate):
        agent_status_msg = AgentStatus()
        agent_status_msg.timestamp = int(self.__clock.now().nanoseconds / 1000)
        agent_status_msg.drone_id = self.drone_id
        agent_status_msg.local_position.x = local_position.x
        agent_status_msg.local_position.y = local_position.y
        agent_status_msg.local_position.z = local_position.z
        agent_status_msg.state = state_name  # e.g., IDLE
        self.status_pub.publish(agent_status_msg)

    def wait_to_drop(self):
        signal_msg = Int8()
        signal_msg.data = self.drone_id
        self.wait_pub.publish(signal_msg)

    # properties
    @property
    def is_ready_to_arm(self):
        return self.__is_ready_to_arm

    @property
    def is_ready_to_takeoff(self):
        return self.__is_ready_to_takeoff

    @property
    def signal(self):
        return self.__signal

    @property
    def supply_zone(self):  # TODO rename (有兩個端點)
        return self.__supply_zone

    @property
    def drop_zone(self):
        return self.__drop_zone

    # callback functions

    def __arm(self, msg: Bool):
        self.__is_ready_to_arm = msg.data

    def __takeoff(self, msg: Bool):
        self.__is_ready_to_takeoff = msg.data

    def __set_signal(self, msg: Empty):
        self.__signal = True

    def __set_supply_zone(self, msg: SupplyZoneInfo):
        self.__supply_zone[0] = NEDCoordinate(
            msg.position_1.x,
            msg.position_1.y,
            msg.position_1.z,
        )
        self.__supply_zone[1] = NEDCoordinate(
            msg.position_2.x,
            msg.position_2.y,
            msg.position_2.z,
        )

    def __set_drop_zone(self, msg: DropZoneInfo):
        self.__drop_zone = msg.position
