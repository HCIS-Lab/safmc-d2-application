# TODO 角度、持續校正 heading

from functools import partial
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool

from agent_msgs.msg import DropZoneInfo, ObstacleArray, SupplyZoneInfo
from common.coordinate import Coordinate
from common.logger import Logger
from mediator.constants import NUM_DRONES


def get_group_id(drone_id: int) -> int:
    return drone_id % 2 + 1


def is_green(drone_id: int) -> bool:
    return drone_id == 1 or drone_id == 2


class Mediator(Node):
    _is_drone_online = [i == 0 for i in range(NUM_DRONES)]  # [True, False, False, ...]
    _is_drone_armed = [i == 0 for i in range(NUM_DRONES)]
    _is_drone_waiting_at_hotspot = [i == 0 for i in range(NUM_DRONES)]
    _has_drop_zone_completed = [i == 0 for i in range(NUM_DRONES)]
    _has_payload_been_taken = [i == 0 for i in range(NUM_DRONES)]
    _model_positions: Dict[str, Optional[Coordinate]] = {
        name: None
        for name in (
            "blue_supply_zone",
            "green_supply_zone",
            *(f"drop_zone_{i}" for i in range(1, 5)),
            *(f"x500_safmc_d2_{i}" for i in range(1, 5)),
        )
    }

    def __init__(self):
        super().__init__("mediator")
        self.logger = Logger(self.get_logger(), self.get_clock())

        # TODO
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriptions
        # 物件位置 (from UWB / Gazebo) -> setter 就先轉成 NED
        for model_name in self._model_positions:  # TODO 跟 UWB 小組商量
            self.create_subscription(
                Point,
                f"/position/{model_name}",
                partial(self._set_model_positions, model_name),
                10,  # TODO
            )

        # Publishers
        self._cmd_takeoff_pubs = [None] * (NUM_DRONES + 1)
        self._cmd_drop_pubs = [None] * (NUM_DRONES + 1)
        self._supply_zone_pubs = [None] * (NUM_DRONES + 1)
        self._drop_zone_pubs = [None] * (NUM_DRONES + 1)
        self._obstacle_array_pubs = [None] * (NUM_DRONES + 1)

        for drone_id in range(1, NUM_DRONES + 1):

            prefix = f"/px4_{drone_id}/mediator/"

            # Online
            self.create_subscription(
                Bool,
                prefix + "online",
                lambda msg: self._set_is_drone_online(msg, drone_id),
                qos_profile,
            )

            # Takeoff
            self._cmd_takeoff_pubs[drone_id] = self.create_publisher(
                Bool, prefix + "cmd_takeoff", qos_profile
            )

            # Anytime
            # TODO[lnfu] status
            self.create_subscription(
                Point,
                prefix + "agent_local_pos",
                lambda msg: self._set_agent_local_pos(
                    msg, drone_id
                ),  # 收到後會回傳 supply/drop zone 位置
                qos_profile,
            )

            self._supply_zone_pubs[drone_id] = self.create_publisher(
                SupplyZoneInfo, prefix + "supply_zone", qos_profile
            )

            self._drop_zone_pubs[drone_id] = self.create_publisher(
                DropZoneInfo, prefix + "drop_zone", qos_profile
            )

            self._obstacle_array_pubs[drone_id] = self.create_publisher(
                ObstacleArray, prefix + "obstacle_array", qos_profile
            )

            # Drop
            self.create_subscription(
                Bool,
                prefix + "drop_request",
                lambda msg: self._set_is_drone_waiting_at_hotspot(msg, drone_id),
                qos_profile,
            )
            self._cmd_drop_pubs[drone_id] = self.create_publisher(
                Bool, prefix + "cmd_drop", qos_profile
            )
            self.create_subscription(
                Bool,
                prefix + "drop_ack",
                lambda msg: self._unset_is_drone_waiting_at_hotspot(msg, drone_id),
                qos_profile,
            )

    def _set_model_positions(self, model_name: str, msg: Point):
        self._model_positions[model_name] = Coordinate.enu_to_ned(
            Coordinate.from_point(msg)
        )

    def _set_is_drone_online(self, msg: Bool, drone_id: int):
        self._is_drone_online[drone_id] = msg

        # Command TAKEOFF
        msg = Bool()
        msg.data = True
        self._cmd_takeoff_pubs[drone_id].publish(msg)

    def _set_is_drone_waiting_at_hotspot(
        self, msg: Bool, drone_id: int
    ):  # TODO refactor
        group_id = get_group_id(drone_id)

        # 標記該無人機已到達熱點
        self._is_drone_waiting_at_hotspot[drone_id] = msg

        if not msg:
            return

        # 檢查同組無人機是否全部到達
        all_group_members_arrived = all(
            self._is_drone_waiting_at_hotspot[other_drone_id]
            for other_drone_id in range(1, NUM_DRONES + 1)
            if get_group_id(other_drone_id) == group_id
        )
        if not all_group_members_arrived:
            return

        # 如果同組無人機都已到達，則發送 DROP 指令
        bool_msg = Bool()
        bool_msg.data = True
        for other_drone_id in range(1, NUM_DRONES + 1):
            if get_group_id(other_drone_id) == group_id:
                self._cmd_drop_pubs[other_drone_id].publish(bool_msg)

    def _get_supply_zone_info_msg(
        self,
        drone_id: int,
        agent_local_position: Coordinate,
        agent_global_position: Coordinate,
        supply_zone_global_position: Coordinate,
    ):
        sign_ = 1 if is_green(drone_id) else -1
        supply_zone_local_position_1 = self._get_target_local_position(
            agent_local_position,
            agent_global_position,
            supply_zone_global_position + sign_ * Coordinate.front * 3,
        )
        supply_zone_local_position_2 = self._get_target_local_position(
            agent_local_position,
            agent_global_position,
            supply_zone_global_position - sign_ * Coordinate.front * 3,
        )
        supply_zone_local_position_3 = self._get_target_local_position(
            agent_local_position,
            agent_global_position,
            supply_zone_global_position
            - sign_ * Coordinate.front * 3
            + Coordinate.right * 1,
        )
        supply_zone_local_position_4 = self._get_target_local_position(
            agent_local_position,
            agent_global_position,
            supply_zone_global_position
            + sign_ * Coordinate.front * 3
            + Coordinate.right * 1,
        )

        sz_msg = SupplyZoneInfo()
        sz_msg.point_1 = supply_zone_local_position_1.to_point()  # near
        sz_msg.point_2 = supply_zone_local_position_2.to_point()  # far
        sz_msg.point_3 = supply_zone_local_position_3.to_point()  # far
        sz_msg.point_4 = supply_zone_local_position_4.to_point()  # near
        sz_msg.aruco_marker_id = 0  # TODO

        return sz_msg

    def _get_drop_zone_info_msg(
        self,
        drone_id: int,
        agent_local_position: Coordinate,
        agent_global_position: Coordinate,
        drop_zone_global_position: Coordinate,
    ):
        dz_msg = DropZoneInfo()
        dz_msg.point = self._get_target_local_position(
            agent_local_position, agent_global_position, drop_zone_global_position
        ).to_point()
        dz_msg.aruco_marker_id = 0  # TODO

        return dz_msg

    def _set_agent_local_pos(self, msg: Point, drone_id: int):

        agent_local_position = Coordinate.from_point(msg)
        agent_global_position: Optional[Coordinate] = self._model_positions[
            f"x500_safmc_d2_{drone_id}"
        ]
        if agent_global_position is None:
            raise Exception()
            # TODO log error

        # Supply Zone Information
        supply_zone_global_position: Optional[Coordinate] = self._model_positions[
            "green_supply_zone" if is_green(drone_id) else "blue_supply_zone"
        ]
        if supply_zone_global_position is None:
            raise Exception()
            # TODO log error

        supply_zone_info_msg = self._get_supply_zone_info_msg(
            drone_id,
            agent_local_position,
            agent_global_position,
            supply_zone_global_position,
        )
        self._supply_zone_pubs[drone_id].publish(supply_zone_info_msg)

        # Drop Zone Information
        group_id = get_group_id(drone_id)

        drop_zone_id_1 = group_id * 2 - 1  # 1 -> 1, 2 -> 3
        drop_zone_id_2 = drop_zone_id_1 + 1  # 1 -> 2, 2 -> 4
        drop_zone_id = (
            drop_zone_id_1
            if not self._has_drop_zone_completed[drop_zone_id_1]
            else drop_zone_id_2
        )

        drop_zone_global_position: Optional[Coordinate] = self._model_positions[
            f"drop_zone_{drop_zone_id}"
        ]
        if drop_zone_global_position is None:
            raise Exception()
            # TODO log error

        drop_zone_info_msg = self._get_drop_zone_info_msg(
            drone_id,
            agent_local_position,
            agent_global_position,
            drop_zone_global_position,
        )
        self._drop_zone_pubs[drone_id].publish(drop_zone_info_msg)

        # Other hidden obstacles
        # ObstacleArray
        obstacle_array_msg = ObstacleArray()
        for other_drone_id in range(1, NUM_DRONES + 1):
            if other_drone_id == drone_id:  # 自己
                continue
            other_global_position = self._model_positions[
                f"x500_safmc_d2_{other_drone_id}"
            ]

            if other_global_position is None:
                continue

            other_local_position = self._get_target_local_position(
                agent_local_position, agent_global_position, other_global_position
            )
            obstacle_array_msg.points.append(other_local_position.to_point())
        self._obstacle_array_pubs[drone_id].publish(obstacle_array_msg)

    def _unset_is_drone_waiting_at_hotspot(self, msg: Bool, drone_id: int):
        if not msg.data:
            return

        group_id = get_group_id(drone_id)

        for other_drone_id in range(1, NUM_DRONES + 1):
            if get_group_id(other_drone_id) == group_id:
                self._is_drone_waiting_at_hotspot[other_drone_id] = False  # 重置

    def _get_target_local_position(
        self,
        agent_local_position: Coordinate,
        agent_global_position: Coordinate,
        target_global_position: Coordinate,
    ):
        assert agent_local_position is not None, f"agent_local_position is None"
        assert agent_global_position is not None, f"agent_global_position is None"
        assert target_global_position is not None, f"target_global_position is None"
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


if __name__ == "__main__":
    main()
