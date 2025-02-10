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


class Mediator(Node):

    def __init__(self):
        super().__init__('mediator')

        self.online = [False, False, False, False]  # 無人機是否在線上
        self.armed = [False, False, False, False]
        self.wait = [False, False, False, False]  # 無人機是否在 Hotspot 正上方等待了
        self.group = [-1, -1, -1, -1]
        self.state = [-1, -1, -1, -1]

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
        self.arm_pubs = [None] * 4
        self.takeoff_pubs = [None] * 4
        self.supply_zone_pubs = [None] * 4
        self.drop_zone_pubs = [None] * 4
        self.drop_pubs = [None] * 4
        for i in range(4):
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

        for i in range(4):
            if self.online[i] == False:
                return

        # 全部都上線了
        bool_msg = Bool()
        bool_msg.data = True
        self.arm_pubs[index].publish(bool_msg)

    def __set_armed(self, agent_info_msg):
        index = agent_info_msg.drone_id - 1
        self.armed[index] = True

        for i in range(4):
            if self.armed[i] == False:
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

        if self.state[index] == 2:  # STATE_HEADING_CALIBRATION
            pass
        elif self.state[index] == 3 or self.state[index] == 4:
            # TODO 持續校正 heading
            # TODO supply zone position (global -> local)
            supply_zone_msg = SupplyZoneInfo()
            if index % 2 == 0:
                green_supply_zone = self.__model_positions["green_supply_zone"]
                supply_zone_msg.position1 = [green_supply_zone.x - 3, green_supply_zone.y, green_supply_zone]
                supply_zone_msg.position2 = [green_supply_zone.x + 3, green_supply_zone.y, green_supply_zone]
            else:
                blue_supply_zone = self.__model_positions["blue_supply_zone"]
                supply_zone_msg.position1 = [blue_supply_zone.x - 3, blue_supply_zone.y, blue_supply_zone]
                supply_zone_msg.position2 = [blue_supply_zone.x + 3, blue_supply_zone.y, blue_supply_zone]
            supply_zone_msg.aruco_marker_id = 0
            self.supply_zone_pubs[index].publish(supply_zone_msg)
        elif self.state[index] == 8 or self.state[index] == 9:
            drop_zone_msg = DropZoneInfo()
            drop_zone_msg.position1 = [0, 0, 0]
            drop_zone_msg.position2 = [0, 0, 0]
            drop_zone_msg.position3 = [0, 0, 0]
            drop_zone_msg.position4 = [0, 0, 0]
            drop_zone_msg.aruco_marker_id = 0
            self.drop_zone_pubs[index].publish(drop_zone_msg)

    def __set_wait(self, agent_info_msg):
        index = agent_info_msg.drone_id - 1
        if self.group[index] != agent_info_msg.group_id:
            # TODO throw error or log error
            return

        self.wait[index] = True

        # 檢查同組的每個人有沒有到
        for i in range(4):
            if self.group[i] == agent_info_msg.group_id and self.wait[i] == False:
                # 同組 (這邊就算是同一台無人機也不用特別 continue)
                return

        # 同組的都到齊了
        for i in range(4):
            if self.group[i] == agent_info_msg.group_id and self.wait[i] == False:
                # 同組 (這邊就算是同一台無人機也不用特別 continue)
                bool_msg = Bool()
                bool_msg.data = True
                self.drop_pubs[i].publish(bool_msg)
                return

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
