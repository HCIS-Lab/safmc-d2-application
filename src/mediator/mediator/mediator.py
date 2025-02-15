# TODO 角度校正
# TODO UWB global -> local
# TODO UWB Supply/Drop Zone
# TODO refactor
# TODO log
# TODO timestamp (如果真的沒用就移除)
# TODO 區分清楚 public/private property

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool

from agent_msgs.msg import AgentInfo, AgentStatus, DropZoneInfo, SupplyZoneInfo
from common.ned_coordinate import NEDCoordinate

from functools import partial

from .constants import DRONE_COUNTS, DROP_ZONE_COUNT


class Mediator(Node):

    def __init__(self):
        super().__init__('mediator')

        self.online = [False] * DRONE_COUNTS  # 無人機是否在線上
        self.armed = [False] * DRONE_COUNTS 
        self.wait = [False] * DRONE_COUNTS   # 無人機是否在 Hotspot 正上方等待了
        self.group = [-1] * DRONE_COUNTS 
        self.state = [-1] * DRONE_COUNTS 
        self.dropzoneDone = [False] * DROP_ZONE_COUNT

        self.__model_positions = {
            "blue_supply_zone": NEDCoordinate(0, 0, 0),
            "green_supply_zone": NEDCoordinate(0, 0, 0),
            "drop_zone_1": NEDCoordinate(0, 0, 0),
            "drop_zone_2": NEDCoordinate(0, 0, 0),
            "drop_zone_3": NEDCoordinate(0, 0, 0),
            "drop_zone_4": NEDCoordinate(0, 0, 0),
            "x500_safmc_d2_1": NEDCoordinate(0, 0, 0),
            "x500_safmc_d2_2": NEDCoordinate(0, 0, 0),
            "x500_safmc_d2_3": NEDCoordinate(0, 0, 0),
            "x500_safmc_d2_4": NEDCoordinate(0, 0, 0)
        }

        # subscriptions
        for model_name in self.__model_positions:
            self.create_subscription(
                Point,
                f"/position/{model_name}",
                partial(self.__set_model_position, model_name),
                10
            )

        self.online_sub = self.create_subscription(
            AgentInfo,
            '/mediator/online',
            self.__set_online,
            10
        )

        self.online_sub = self.create_subscription(
            AgentInfo,
            '/mediator/armed',
            self.__set_armed,
            10
        )

        self.status_sub = self.create_subscription(
            AgentStatus,
            '/mediator/status',
            self.__set_status,
            10
        )

        self.wait_sub = self.create_subscription(
            AgentInfo,
            '/mediator/wait',
            self.__set_wait,
            10
        )

        self.drop_ack_sub = self.create_subscription(
            AgentInfo,
            f"/mediator/drop_ack",
            self.__set_drop_ack,
            10
        )

        # publishers
        self.arm_pubs = [None] * DRONE_COUNTS 
        self.takeoff_pubs = [None] * DRONE_COUNTS 
        self.supply_zone_pubs = [None] * DRONE_COUNTS 
        self.drop_zone_pubs = [None] * DRONE_COUNTS 
        self.drop_pubs = [None] * DRONE_COUNTS 
        for i in range(DRONE_COUNTS):
            self.arm_pubs[i] = self.create_publisher(
                Bool,
                f"/agent_{i+1}/arm",
                10
            )

            self.takeoff_pubs[i] = self.create_publisher(
                Bool,
                f"/agent_{i+1}/takeoff",
                10
            )

            self.supply_zone_pubs[i] = self.create_publisher(
                SupplyZoneInfo,
                f"/agent_{i+1}/supply_zone",
                10
            )

            self.drop_zone_pubs[i] = self.create_publisher(
                DropZoneInfo,
                f"/agent_{i+1}/drop_zone",
                10
            )

            self.drop_pubs[i] = self.create_publisher(
                Bool,
                f"/agent_{i+1}/drop",
                10
            )

    def __set_model_position(self, model_name: str, msg: Point):
        self.__model_positions[model_name].x = msg.x
        self.__model_positions[model_name].y = msg.y
        self.__model_positions[model_name].z = -msg.z

    def __set_online(self, agent_info_msg):
        index = agent_info_msg.drone_id - 1
        self.online[index] = True
        self.group[index] = agent_info_msg.group_id

        # for i in range(4):
        #     if self.online[i] == False:
        #         return
        if not all(self.online):
            return

        # 全部都上線了
        bool_msg = Bool()
        bool_msg.data = True
        self.arm_pubs[index].publish(bool_msg)

    def __set_armed(self, agent_info_msg):
        index = agent_info_msg.drone_id - 1
        self.armed[index] = True

        # for i in range(4):
        #     if self.armed[i] == False:
        #         return
        if not all(self.armed):
            return

        # 全部都armed了
        bool_msg = Bool()
        bool_msg.data = True
        self.takeoff_pubs[index].publish(bool_msg)

    def __set_status(self, agent_status_msg):
        index = agent_status_msg.drone_id - 1
        if self.group[index] != agent_status_msg.group_id:
            # TODO throw error or log error
            return

        # TODO: timestamp???

        self.state[index] = agent_status_msg.state
        
        # supply zone

        # TODO 持續校正 heading
        # TODO supply zone position (global -> local)
        supply_zone_msg = SupplyZoneInfo()
        if index % 2 == 0:
            green_supply_zone = self.__model_positions["green_supply_zone"]
            supply_zone_msg.position_1 = [green_supply_zone.y, green_supply_zone.x - 3, green_supply_zone.z]
            supply_zone_msg.position_2 = [green_supply_zone.y, green_supply_zone.x + 3, green_supply_zone.z]
        else:
            blue_supply_zone = self.__model_positions["blue_supply_zone"]
            supply_zone_msg.position_1 = [blue_supply_zone.y, blue_supply_zone.x - 3, blue_supply_zone.z]
            supply_zone_msg.position_2 = [blue_supply_zone.y, blue_supply_zone.x + 3, blue_supply_zone.z]
        supply_zone_msg.aruco_marker_id = 0
        self.supply_zone_pubs[index].publish(supply_zone_msg)
        
        # drop zone
        drop_zone_msg = DropZoneInfo()
        drop_zone = [self.__model_positions[f"drop_zone_{i+1}"] for i in range(DROP_ZONE_COUNT)]
        if index < 2:
            if self.dropzoneDone[0] == True:
                drop_zone_msg.position = [drop_zone[1].y, drop_zone[1].x, drop_zone[1].z]
            else:
                drop_zone_msg.position = [drop_zone[0].y, drop_zone[0].x, drop_zone[0].z]

        else:
            if self.dropzoneDone[2] == True:
                drop_zone_msg.position = [drop_zone[3].y, drop_zone[3].x, drop_zone[3].z]
            else:
                drop_zone_msg.position = [drop_zone[2].y, drop_zone[2].x, drop_zone[2].z]

        drop_zone_msg.aruco_marker_id = 0
        self.drop_zone_pubs[index].publish(drop_zone_msg)

    def __set_wait(self, agent_info_msg):
        index = agent_info_msg.drone_id - 1
        if self.group[index] != agent_info_msg.group_id:
            # TODO throw error or log error
            return

        self.wait[index] = True

        # 檢查同組的每個人有沒有到
        # for i in range(DRONE_COUNTS):
        #     if self.group[i] == agent_info_msg.group_id and self.wait[i] == False:
        #         # 同組 (這邊就算是同一台無人機也不用特別 continue)
        #         return
        if not all((self.group[i] == agent_info_msg.group_id and self.wait[i] == False
                    for i in range(DRONE_COUNTS))):
            return

        # 同組的都到齊了
        for i in range(DRONE_COUNTS):
            if self.group[i] == agent_info_msg.group_id:
                # 同組 (這邊就算是同一台無人機也不用特別 continue)
                bool_msg = Bool()
                bool_msg.data = True
                self.drop_pubs[i].publish(bool_msg)


    def __set_drop_ack(self, agent_info_msg):
        index = agent_info_msg.drone_id - 1
        if self.group[index] != agent_info_msg.group_id:
            # TODO throw error or log error
            return

        self.wait[index] = False  # 重置


def main(args=None):

    rclpy.init(args=args)
    mediator = Mediator()

    try:
        rclpy.spin(mediator)
    finally:
        mediator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
