# TODO 角度校正
# TODO 持續校正 heading

# drone_id % 2 == 0 -> Green Supply Zone
# drone_id % 2 == 1 -> Blue Supply Zone

# Group 1: 1, 2 (drone_id)
# Group 2: 3, 4 (drone_id)

from functools import partial
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import Bool

from agent_msgs.msg import AgentInfo, AgentStatus, DropZoneInfo, SupplyZoneInfo
from common.coordinate import Coordinate
from common.logger import Logger
from mediator.constants import NUM_DRONES, NUM_DROP_ZONES, NUM_PAYLOADS


def get_group_id(drone_id: int) -> int:
    if drone_id == 1 or drone_id == 2:
        return 1
    return 2


def is_green(drone_id: int) -> bool:
    return drone_id % 2 == 0


def init_status_flags(size: int):
    flags = [False] * (size + 1)
    flags[0] = True
    return flags


class Mediator(Node):

    def __init__(self):
        super().__init__('mediator')
        self.logger = Logger(self.get_logger(), self.get_clock())

        self.is_drone_online = init_status_flags(NUM_DRONES)
        self.is_drone_armed = init_status_flags(NUM_DRONES)
        self.is_drone_waiting_at_hotspot = init_status_flags(NUM_DRONES)
        self.has_drop_zone_completed = init_status_flags(NUM_DROP_ZONES)
        self.has_payload_been_taken = init_status_flags(NUM_PAYLOADS)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.create_subscription(AgentInfo, '/mediator/online', self.__set_is_drone_online, qos_profile)
        self.create_subscription(AgentInfo, '/mediator/arm_ack', self.__set_is_drone_armed, qos_profile)
        self.create_subscription(AgentStatus, '/mediator/status', self.__set_status, qos_profile)
        self.create_subscription(AgentInfo, '/mediator/wait', self.__set_wait_list, qos_profile)
        self.create_subscription(AgentInfo, f"/mediator/drop_ack", self.__set_drop_ack, qos_profile)

        # 物件位置 (from UWB / Gazebo) -> setter 就先轉成 NED
        self.__model_positions: Dict[str, Optional[Coordinate]] = {
            name: None for name in (
                "blue_supply_zone", "green_supply_zone",
                *(f"drop_zone_{i}" for i in range(1, 5)),
                *(f"x500_safmc_d2_{i}" for i in range(1, 5))
            )
        }
        for model_name in self.__model_positions:
            self.create_subscription(Point, f"/position/{model_name}",
                                     partial(self.__set_model_positions, model_name), 10)

        # publishers
        self.arm_pubs = [None] * (NUM_DRONES+1)
        self.takeoff_pubs = [None] * (NUM_DRONES+1)
        self.supply_zone_info_pubs = [None] * (NUM_DRONES+1)
        self.drop_zone_info_pubs = [None] * (NUM_DRONES+1)
        self.drop_pubs = [None] * (NUM_DRONES+1)

        for drone_id in range(1, NUM_DRONES+1):
            self.arm_pubs[drone_id] = self.create_publisher(
                Bool,
                f"/agent_{drone_id+1}/arm",
                qos_profile
            )

            self.takeoff_pubs[drone_id] = self.create_publisher(
                Bool,
                f"/agent_{drone_id+1}/takeoff",
                qos_profile
            )

            self.supply_zone_info_pubs[drone_id] = self.create_publisher(
                SupplyZoneInfo,
                f"/agent_{drone_id+1}/supply_zone",
                qos_profile
            )

            self.drop_zone_info_pubs[drone_id] = self.create_publisher(
                DropZoneInfo,
                f"/agent_{drone_id+1}/drop_zone",
                qos_profile
            )

            self.drop_pubs[drone_id] = self.create_publisher(
                Bool,
                f"/agent_{drone_id+1}/drop",
                qos_profile
            )

    def __set_model_positions(self, model_name: str, msg: Point):
        self.__model_positions[model_name] = Coordinate.enu_to_ned(Coordinate.from_point(msg))

    def __set_is_drone_online(self, agent_info_msg):
        drone_id = agent_info_msg.drone_id
        if not self.is_drone_online[drone_id]:
            self.logger.ori.info(f"{drone_id} is online!")
        self.is_drone_online[drone_id] = True

        self.logger.info("sending arming command")
        msg = Bool()
        msg.data = True
        self.arm_pubs[drone_id].publish(msg)

    def __set_is_drone_armed(self, agent_info_msg):
        drone_id = agent_info_msg.drone_id
        if not self.is_drone_armed[drone_id]:
            self.logger.ori.info(f"{drone_id} is armed!")
        self.is_drone_armed[drone_id] = True

        self.logger.info("sending takeoff command")
        msg = Bool()
        msg.data = True
        self.takeoff_pubs[drone_id].publish(msg)

    def __set_status(self, agent_status_msg):
        drone_id = agent_status_msg.drone_id
        group_id = get_group_id(drone_id)

        agent_local_position = Coordinate(
            agent_status_msg.local_position.x,
            agent_status_msg.local_position.y,
            agent_status_msg.local_position.z,
        )
        agent_global_position: Optional[Coordinate] = self.__model_positions[f"x500_safmc_d2_{drone_id}"]
        if agent_global_position is None:
            return

        # Supply Zone Information
        supply_zone_name = "green_supply_zone" if is_green(drone_id) else "blue_supply_zone"
        supply_zone_global_position: Optional[Coordinate] = self.__model_positions[supply_zone_name]
        if supply_zone_global_position is None:
            return

        supply_zone_local_position = self.get_target_local_position(
            agent_local_position,
            agent_global_position,
            supply_zone_global_position
        )

        supply_zone_msg = SupplyZoneInfo()
        supply_zone_msg.position_1 = (supply_zone_local_position + Coordinate.front * 3).to_point()
        supply_zone_msg.position_2 = (supply_zone_local_position - Coordinate.front * 3).to_point()
        supply_zone_msg.aruco_marker_id = 0  # TODO
        self.supply_zone_info_pubs[drone_id].publish(supply_zone_msg)

        # Drop Zone Information
        drop_zone_id_1 = group_id * 2 - 1  # 1 -> 1, 2 -> 3
        drop_zone_id_2 = drop_zone_id_1 + 1  # 1 -> 2, 2 -> 4
        drop_zone_id = drop_zone_id_1 if not self.has_drop_zone_completed[drop_zone_id_1] else drop_zone_id_2
        drop_zone_global_position: Optional[Coordinate] = self.__model_positions[f"drop_zone_{drop_zone_id}"]

        drop_zone_msg = DropZoneInfo()
        drop_zone_msg.position = self.get_target_local_position(
            agent_local_position,
            agent_global_position,
            drop_zone_global_position
        ).to_point()
        drop_zone_msg.aruco_marker_id = 0  # TODO
        self.drop_zone_info_pubs[drone_id].publish(drop_zone_msg)

    def __set_wait_list(self, agent_info_msg):
        drone_id = agent_info_msg.drone_id
        group_id = get_group_id(drone_id)

        self.is_drone_waiting_at_hotspot[drone_id] = True

        # 檢查同組的每個人有沒有到
        if any(
            self.__group_id(i + 1) == group_id and not self.is_drone_waiting_at_hotspot[i]
            for i in range(1, NUM_DRONES+1)
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

        for i in range(NUM_DRONES):
            if self.__group_id(i + 1) == group_id:
                self.is_drone_waiting_at_hotspot[drone_id] = False  # 重置

    def get_target_local_position(self, agent_local_position: Coordinate, agent_global_position: Coordinate, target_global_position: Coordinate):
        return Coordinate(
            agent_local_position.x + target_global_position.x - agent_global_position.x,
            agent_local_position.y + target_global_position.y - agent_global_position.y,
            agent_local_position.z + target_global_position.z - agent_global_position.z,
        )


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
