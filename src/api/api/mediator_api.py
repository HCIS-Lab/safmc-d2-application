from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Empty, Int8, Bool
from agent_msgs.msg import AgentInfo, AgentStatus, DropZoneInfo, SupplyZoneInfo

from .api import Api


class MediatorApi(Api):
    def __init__(self, node: Node, drone_id: int, group_id: int):

        self.drone_id = drone_id
        self.group_id = group_id

        self.__clock: Clock = node.get_clock()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.__signal = False
        self.__supply_zone = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.__takeoff_ready = False
        self.__arm_ready = False
        self.signal_sub = node.create_subscription(
            Empty, f'/group{self.group_id}/signal', self.__set_signal, qos_profile)
        
        self.supply_zone_sub = node.create_subscription(
            SupplyZoneInfo,
            f"/agent_{self.drone_id}/supply_zone",
            self.__set_supply_zone,
            10
        )

        self.arm_sub = node.create_subscription(
            Bool,
            f"/agent_{self.drone_id}/arm",
            self.__arm,
            10
        )

        self.takeoff_sub = node.create_subscription(
            Bool,
            f"/agent_{self.drone_id}/takeoff",
            self.__takeoff,
            10
        )

        # Publishers
        
        
        self.online_pub = node.create_publisher(
            AgentInfo,
            '/mediator/online',
            10
        )

        self.armed_pub = node.create_publisher(
            AgentInfo,
            '/mediator/armed',
            10
        )
        
        self.wait_pub = node.create_publisher(
            Int8, f'/group{self.group_id}/wait', qos_profile)

    
    # api functions
    def online(self):
        online_msg = AgentInfo()
        online_msg.timestamp = int(self.__clock.now().nanoseconds / 1000)
        online_msg.drone_id = self.drone_id
        online_msg.group_id = self.group_id
        self.online_pub.publish(online_msg)

    def armed(self):
        armed_msg = AgentInfo()
        armed_msg.timestamp = int(self.__clock.now().nanoseconds / 1000)
        armed_msg.drone_id = self.drone_id
        armed_msg.group_id = self.group_id
        self.armed_pub.publish(armed_msg)
        
    def wait_to_drop(self):
        signal_msg = Int8()
        signal_msg.data = self.drone_id
        self.wait_pub.publish(signal_msg)

    # properties
    @property
    def arm_ready(self):
        return self.__arm_ready
    @property
    def takeoff_ready(self):
        return self.__takeoff_ready  
    @property
    def signal(self):
        return self.__signal
    @property
    def supply_zone(self):
        return self.__supply_zone
    
    

    # callback functions
    def __arm(self, msg: Bool):
        self.__arm_ready = msg
    
    def __takeoff(self, msg: Bool):
        self.__takeoff_ready = msg

    def __set_signal(self, msg: Empty):
        self.__signal = True

    def __set_supply_zone(self, msg: SupplyZoneInfo):
        self.__supply_zone[0] = msg.position_1
        self.__supply_zone[1] = msg.position_2