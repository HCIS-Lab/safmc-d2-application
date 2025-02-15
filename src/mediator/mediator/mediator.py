# TODO 角度校正

# drone_id % 2 == 0 -> Green Supply Zone
# drone_id % 2 == 1 -> Blue Supply Zone

# Group 1: 1, 2 (drone_id)
# Group 2: 3, 4 (drone_id)

from functools import partial

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool

from agent_msgs.msg import AgentInfo, AgentStatus, DropZoneInfo, SupplyZoneInfo
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .constants import NUM_DRONES, NUM_DROP_ZONES


def drone_id_to_index(drone_id: int) -> int:
    """ Convert 1-indexed drone_id to 0-indexed list index """
    return drone_id - 1


def index_to_drone_id(index: int) -> int:
    """ Convert 0-indexed list index to 1-indexed drone_id """
    return index + 1


def get_group_id(drone_id: int) -> int:
    """ 無人機群組 1-index: 1,2 / 3,4 """
    if drone_id == 1 or drone_id == 2:
        return 1
    return 2


def is_green(drone_id: int) -> bool:
    """ drone_id 偶數屬於 Green Supply Zone, 奇數屬於 Blue Supply Zone """
    return drone_id % 2 == 0


class Mediator(Node):

    def __init__(self):
        super().__init__('mediator')
        self.logger = Logger(self.get_logger(), self.get_clock())

        self.online_list = [False] * NUM_DRONES  # 無人機是否在線上
        self.is_armed_list = [False] * NUM_DRONES  # 無人機是否已經 arming
        self.wait_list = [False] * NUM_DRONES   # 無人機是否在 Hotspot 正上方等待了
        self.is_drop_zone_done = [False] * NUM_DROP_ZONES  # drop zone 已經完成

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions

        self.create_subscription(
            AgentInfo,
            '/mediator/online',
            self.__set_online_and_group_list,
            qos_profile
        )

        self.create_subscription(
            AgentInfo,
            '/mediator/arm_ack',
            self.__set_is_armed_list,
            qos_profile
        )

        self.create_subscription(
            AgentStatus,
            '/mediator/status',
            self.__set_status,
            qos_profile
        )

        self.wait_sub = self.create_subscription(
            AgentInfo,
            '/mediator/wait',
            self.__set_wait_list,
            qos_profile
        )

        self.drop_ack_sub = self.create_subscription(
            AgentInfo,
            f"/mediator/drop_ack",
            self.__set_drop_ack,
            qos_profile
        )

        # 物件位置 (from UWB / Gazebo)
        self.__model_positions = {
            "blue_supply_zone": None,
            "green_supply_zone": None,
            "drop_zone_1": None,
            "drop_zone_2": None,
            "drop_zone_3": None,
            "drop_zone_4": None,
            "x500_safmc_d2_1": None,
            "x500_safmc_d2_2": None,
            "x500_safmc_d2_3": None,
            "x500_safmc_d2_4": None,
        }

        for model_name in self.__model_positions:
            self.create_subscription(
                Point,
                f"/position/{model_name}",
                partial(self.__set_model_position, model_name),
                10  # TODO
            )

        # publishers
        self.arm_pubs = [None] * NUM_DRONES
        self.takeoff_pubs = [None] * NUM_DRONES

        self.supply_zone_info_pubs = [None] * NUM_DRONES
        self.drop_zone_info_pubs = [None] * NUM_DRONES

        self.drop_pubs = [None] * NUM_DRONES

        for i in range(NUM_DRONES):
            self.arm_pubs[i] = self.create_publisher(
                Bool,
                f"/agent_{i+1}/arm",
                qos_profile
            )

            self.takeoff_pubs[i] = self.create_publisher(
                Bool,
                f"/agent_{i+1}/takeoff",
                qos_profile
            )

            self.supply_zone_info_pubs[i] = self.create_publisher(
                SupplyZoneInfo,
                f"/agent_{i+1}/supply_zone",
                qos_profile
            )

            self.drop_zone_info_pubs[i] = self.create_publisher(
                DropZoneInfo,
                f"/agent_{i+1}/drop_zone",
                qos_profile
            )

            self.drop_pubs[i] = self.create_publisher(
                Bool,
                f"/agent_{i+1}/drop",
                qos_profile
            )

    def __set_model_position(self, model_name: str, msg: Point):
        self.__model_positions[model_name] = NEDCoordinate(
            msg.x,
            msg.y,
            msg.z
        )

    def __set_online_and_group_list(self, agent_info_msg):
        drone_id = agent_info_msg.drone_id
        group_id = get_group_id(drone_id)
        index = drone_id_to_index(drone_id)

        self.online_list[index] = True

        # 還有人沒有上線
        if not all(self.online_list):
            return

        # 全部都上線了, 讓同所有人都 arming
        for i in range(NUM_DRONES):
            bool_msg = Bool()
            bool_msg.data = True
            self.arm_pubs[i].publish(bool_msg)

    def __set_is_armed_list(self, agent_info_msg):
        drone_id = agent_info_msg.drone_id
        group_id = get_group_id(drone_id)
        index = drone_id_to_index(drone_id)

        self.is_armed_list[index] = True

        # 還有人沒有 arming
        if not all(self.is_armed_list):
            return

        # 全部都 armed 了, 讓所有人起飛
        for i in range(NUM_DRONES):
            bool_msg = Bool()
            bool_msg.data = True
            self.takeoff_pubs[i].publish(bool_msg)

    def __set_status(self, agent_status_msg):
        drone_id = agent_status_msg.drone_id
        group_id = get_group_id(drone_id)
        index = drone_id_to_index(drone_id)

        agent_local_position = NEDCoordinate(
            agent_status_msg.local_position.x,
            agent_status_msg.local_position.y,
            agent_status_msg.local_position.z,
        )
        agent_global_position: NEDCoordinate = self.__model_positions[f"x500_safmc_d2_{drone_id}"]

        # TODO 持續校正 heading

        # Supply Zone Information

        supply_zone_msg = self.__get_supply_zone_position(
            drone_id,
            agent_local_position,
            agent_global_position,
            self.__model_positions
        )
        if supply_zone_msg:
            supply_zone_msg.aruco_marker_id = 0  # TODO
            self.supply_zone_info_pubs[index].publish(supply_zone_msg)

        # Drop Zone Information
        drop_zone_msg = self.__get_drop_zone_position(
            drone_id,
            agent_local_position,
            agent_global_position,
            self.__model_positions,
            self.is_drop_zone_done
        )
        if drop_zone_msg:
            drop_zone_msg.aruco_marker_id = 0  # TODO
            self.drop_zone_info_pubs[index].publish(drop_zone_msg)

    def __set_wait_list(self, agent_info_msg):
        drone_id = agent_info_msg.drone_id
        group_id = get_group_id(drone_id)
        index = drone_id_to_index(drone_id)

        self.wait_list[index] = True

        # 檢查同組的每個人有沒有到
        if any(
            self.__group_id(i + 1) == group_id and not self.wait_list[i]
            for i in range(NUM_DRONES)
        ):
            return

        # 同組的都到齊了, 丟下 payload
        for i in range(NUM_DRONES):
            if self.__group_id(i + 1) == group_id:
                bool_msg = Bool()
                bool_msg.data = True
                self.drop_pubs[i].publish(bool_msg)

    def __set_drop_ack(self, agent_info_msg):
        drone_id = agent_info_msg.drone_id
        group_id = get_group_id(drone_id)
        index = drone_id_to_index(drone_id)

        for i in range(NUM_DRONES):
            if self.__group_id(i + 1) == group_id:
                self.wait_list[index] = False  # 重置

    def __get_target_local_position(self, agent_local_position: NEDCoordinate, agent_global_position: NEDCoordinate, target_global_position: NEDCoordinate):
        # TODO 真的要 x, y 互換?
        target_local_position = NEDCoordinate(0, 0, 0)
        target_local_position.x = agent_local_position.x + (target_global_position.y - agent_global_position.x)
        target_local_position.x = agent_local_position.y + (target_global_position.x - agent_global_position.y)
        target_local_position.z = agent_local_position.z + (target_global_position.z - agent_global_position.z)
        return target_local_position

    def __get_supply_zone_position(self, drone_id: int, agent_local: NEDCoordinate, agent_global: NEDCoordinate, positions: dict) -> SupplyZoneInfo:
        """ 計算無人機的 Supply Zone 座標 """
        supply_zone_key = "green_supply_zone" if is_green(drone_id) else "blue_supply_zone"

        if positions[supply_zone_key] == None:
            return None

        supply_zone_msg = SupplyZoneInfo()

        pos1 = self.__get_target_local_position(
            agent_local,
            agent_global,
            positions[supply_zone_key] - NEDCoordinate.north * 3
        )
        supply_zone_msg.position_1.x = float(pos1.x)
        supply_zone_msg.position_1.y = float(pos1.y)
        supply_zone_msg.position_1.z = float(pos1.z)

        pos2 = self.__get_target_local_position(
            agent_local,
            agent_global,
            positions[supply_zone_key] + NEDCoordinate.north * 3
        )
        supply_zone_msg.position_2.x = float(pos2.x)
        supply_zone_msg.position_2.y = float(pos2.y)
        supply_zone_msg.position_2.z = float(pos2.z)

        return supply_zone_msg

    def __get_drop_zone_position(self, drone_id: int, agent_local: NEDCoordinate, agent_global: NEDCoordinate, positions: dict, is_done: list) -> DropZoneInfo:
        """ 計算無人機的 Drop Zone 座標 """
        group_id = get_group_id(drone_id)

        drop_zone_msg = DropZoneInfo()

        drop_zone_idx_1 = (group_id - 1) * 2  # 0 或 2
        drop_zone_idx_2 = drop_zone_idx_1 + 1  # 1 或 3

        target_drop_zone_idx = drop_zone_idx_1 if not is_done[drop_zone_idx_1] else drop_zone_idx_2

        if positions[f"drop_zone_{target_drop_zone_idx + 1}"] == None:
            return None

        pos = self.__get_target_local_position(
            agent_local,
            agent_global,
            positions[f"drop_zone_{target_drop_zone_idx + 1}"] + NEDCoordinate.north * 3
        )
        drop_zone_msg.position.x = float(pos.x)
        drop_zone_msg.position.y = float(pos.y)
        drop_zone_msg.position.z = float(pos.z)
        return drop_zone_msg


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
