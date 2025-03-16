from functools import partial

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from safmc_msgs.msg import TagPosition
from std_msgs.msg import Bool

from safmc_msgs.msg import DropZoneInfo, ObstacleArray, SupplyZoneInfo
from common.coordinate import Coordinate
from common.logger import Logger
from common.qos import cmd_qos_profile
from mediator.constants import NUM_DRONES


schedule = {
    1: {
        "color": "green",
        "teammate": 2,
        "task1": {"supply_marker_id": 0, "drop_zone": "A", "drop_marker_id": 8},
        "task2": {"supply_marker_id": 1, "drop_zone": "B", "drop_marker_id": 9},
    },
    2: {
        "color": "blue",
        "teammate": 1,
        "task1": {"supply_marker_id": 2, "drop_zone": "A", "drop_marker_id": 10},
        "task2": {"supply_marker_id": 3, "drop_zone": "B", "drop_marker_id": 11},
    },
    3: {
        "color": "green",
        "teammate": 4,
        "task1": {"supply_marker_id": 4, "drop_zone": "C", "drop_marker_id": 12},
        "task2": {"supply_marker_id": 5, "drop_zone": "D", "drop_marker_id": 13},
    },
    4: {
        "color": "blue",
        "teammate": 3,
        "task1": {"supply_marker_id": 6, "drop_zone": "C", "drop_marker_id": 14},
        "task2": {"supply_marker_id": 7, "drop_zone": "D", "drop_marker_id": 15},
    },
}


class Mediator(Node):
    _drop_zone_global_p = {"A": None, "B": None, "C": None, "D": None}
    _supply_zone_global_p = {"green": None, "blue": None}
    _agent_global_p = {1: None, 2: None, 3: None, 4: None}
    _agent_task = {1: "task1", 2: "task1", 3: "task1", 4: "task1"}

    # 是否再 hotstop 等待隊友
    _is_drone_waiting_at_hotspot = [False] * (NUM_DRONES + 1)

    # Publishers
    _cmd_takeoff_pubs = [None] * (NUM_DRONES + 1)
    _cmd_drop_pubs = [None] * (NUM_DRONES + 1)
    _supply_zone_pubs = [None] * (NUM_DRONES + 1)
    _drop_zone_pubs = [None] * (NUM_DRONES + 1)
    _obstacle_array_pubs = [None] * (NUM_DRONES + 1)

    def __init__(self):
        super().__init__("mediator")
        Logger(self)

        self.create_subscription(
            TagPosition, "/tag_position", self._set_tag_posision, cmd_qos_profile
        )

        for drone_id in range(1, NUM_DRONES + 1):

            prefix = f"/px4_{drone_id}/mediator/"

            # Online
            self.create_subscription(
                Bool,
                prefix + "online",
                partial(self._set_is_drone_online, drone_id=drone_id),
                cmd_qos_profile,
            )

            # Takeoff
            self._cmd_takeoff_pubs[drone_id] = self.create_publisher(
                Bool, prefix + "cmd_takeoff", cmd_qos_profile
            )

            # 每次收到 agent_local_p 後會回傳 supply/drop zone 位置以及其他飛機的位置
            # TODO[lnfu] 理論上還要回傳場地邊界的位置
            self.create_subscription(
                Point,
                prefix + "agent_local_p",
                partial(self._set_agent_local_p, drone_id=drone_id),
                cmd_qos_profile,
            )

            self._supply_zone_pubs[drone_id] = self.create_publisher(
                SupplyZoneInfo, prefix + "supply_zone", cmd_qos_profile
            )

            self._drop_zone_pubs[drone_id] = self.create_publisher(
                DropZoneInfo, prefix + "drop_zone", cmd_qos_profile
            )

            self._obstacle_array_pubs[drone_id] = self.create_publisher(
                ObstacleArray, prefix + "obstacle_array", cmd_qos_profile
            )

            # Drop
            self.create_subscription(
                Bool,
                prefix + "drop_request",
                partial(self._set_is_drone_waiting_at_hotspot, drone_id=drone_id),
                cmd_qos_profile,
            )
            self._cmd_drop_pubs[drone_id] = self.create_publisher(
                Bool, prefix + "cmd_drop", cmd_qos_profile
            )
            self.create_subscription(
                Bool,
                prefix + "drop_ack",
                partial(self._unset_is_drone_waiting_at_hotspot, drone_id=drone_id),
                cmd_qos_profile,
            )

    def _set_tag_posision(self, msg):
        """記下 UWB 傳來的資料"""
        if msg.eui == "01:01":
            self._agent_global_p[1] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "02:02":
            self._agent_global_p[2] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "03:03":
            self._agent_global_p[3] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "04:04":
            self._agent_global_p[4] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "05:05":
            self._supply_zone_global_p["green"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "06:06":
            self._supply_zone_global_p["blue"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "07:07":
            self._drop_zone_global_p["A"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "08:08":
            self._drop_zone_global_p["B"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "09:09":
            self._drop_zone_global_p["C"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "10:10":
            self._drop_zone_global_p["D"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        else:
            raise ValueError()

    def _set_is_drone_online(self, msg: Bool, drone_id: int):
        """收到上線通知, 回傳起飛指令"""
        if msg.data:
            Logger.info(f"DRONE {drone_id} IS ONLINE!")

            # 發送 TAKEOFF
            msg = Bool()
            msg.data = True
            self._cmd_takeoff_pubs[drone_id].publish(msg)

    def _set_agent_local_p(self, msg: Point, drone_id: int):
        """
        接收 agent local position

        回傳:
        1. supply zone local position
        2. drop zone local position
        3. hidden obstacle(s) local position(s)
        """

        agent_local_p = Coordinate.from_point(msg)
        agent_global_p = self._agent_global_p[drone_id]

        if agent_global_p is None:
            Logger.warning(f"尚未收到 UWB 資訊 AGENT_ID = {drone_id}")
            return

        task = self._agent_task[drone_id]

        # 1. 傳送 Supply Zone 位置
        color = schedule[drone_id]["color"]  # "green" | "blue"
        if color == "green":
            supply_zone_global_p = self._supply_zone_global_p["green"]
            sign_ = 1  # 計算 supply zone 四個端點位置需要
        elif color == "blue":
            supply_zone_global_p = self._supply_zone_global_p["blue"]
            sign_ = -1  # 計算 supply zone 四個端點位置需要
        else:
            raise ValueError()
        if supply_zone_global_p is None:
            Logger.warning(f"尚未收到 UWB 資訊 SUPPLY_COLOR = {color}")
            return
        sz_msg = SupplyZoneInfo()
        sz_msg.point_1 = self._get_target_local_position(
            agent_local_p,
            agent_global_p,
            supply_zone_global_p + sign_ * Coordinate.front * 3,
        ).to_point()
        sz_msg.point_2 = self._get_target_local_position(
            agent_local_p,
            agent_global_p,
            supply_zone_global_p - sign_ * Coordinate.front * 3,
        ).to_point()
        sz_msg.point_3 = self._get_target_local_position(
            agent_local_p,
            agent_global_p,
            supply_zone_global_p - sign_ * Coordinate.front * 3 + Coordinate.right * 1,
        ).to_point()
        sz_msg.point_4 = self._get_target_local_position(
            agent_local_p,
            agent_global_p,
            supply_zone_global_p + sign_ * Coordinate.front * 3 + Coordinate.right * 1,
        ).to_point()
        sz_msg.aruco_marker_id = schedule[drone_id][task]["supply_marker_id"]
        self._supply_zone_pubs[drone_id].publish(sz_msg)

        # 2. 傳送 Drop Zone 位置
        drop_zone_id = schedule[drone_id][task]["drop_zone"]  # "A" | "B" | "C" | "D"
        drop_zone_global_p = self._drop_zone_global_p[drop_zone_id]
        if drop_zone_global_p is None:
            Logger.warning(f"尚未收到 UWB 資訊 DROP_ZONE_ID = {drop_zone_id}")
            return
        dz_msg = DropZoneInfo()
        dz_msg.point = self._get_target_local_position(
            agent_local_p, agent_global_p, drop_zone_global_p
        ).to_point()
        dz_msg.aruco_marker_id = schedule[drone_id][task]["drop_marker_id"]
        self._drop_zone_pubs[drone_id].publish(dz_msg)

        # 3. 傳送 hidden obstacle 位置
        oa_msg = ObstacleArray()
        for other_id in range(1, NUM_DRONES + 1):
            if other_id == drone_id:  # 自己跳過
                continue
            other_global_p = self._agent_global_p[other_id]
            if other_global_p is None:
                Logger.warning(f"尚未收到 UWB 資訊 AGENT_ID = {other_id}")
                return
            other_local_p = self._get_target_local_position(
                agent_local_p, agent_global_p, other_global_p
            )
            oa_msg.points.append(other_local_p.to_point())
        self._obstacle_array_pubs[drone_id].publish(oa_msg)

    def _set_is_drone_waiting_at_hotspot(self, msg: Bool, drone_id: int):

        # 標記該無人機已到達 hotspot
        self._is_drone_waiting_at_hotspot[drone_id] = msg.data

        if not self._is_drone_waiting_at_hotspot[drone_id]:
            return

        # 檢查隊友是否到達
        teammate_id = schedule[drone_id]["teammate"]
        if not self._is_drone_waiting_at_hotspot[teammate_id]:
            Logger.info(
                f"隊友還沒到達 DRONE_ID = {drone_id}, TEAMMATE_ID = {teammate_id}"
            )
            return

        # 組員到達, 同時發送 DROP 指令
        Logger.info(f"同組到達 DRONE_ID = {drone_id}, TEAMMATE_ID = {teammate_id}")
        bool_msg = Bool()
        bool_msg.data = True
        self._cmd_drop_pubs[drone_id].publish(bool_msg)
        self._cmd_drop_pubs[teammate_id].publish(bool_msg)

    def _unset_is_drone_waiting_at_hotspot(self, msg: Bool, drone_id: int):
        if not msg.data:
            return

        teammate_id = schedule[drone_id]["teammate"]

        self._is_drone_waiting_at_hotspot[drone_id] = False
        self._is_drone_waiting_at_hotspot[teammate_id] = False

        self._agent_task[drone_id] = "task2"
        self._agent_task[teammate_id] = "task2"

    def _get_target_local_position(
        self,
        agent_local_position: Coordinate,
        agent_global_position: Coordinate,
        target_global_position: Coordinate,
    ):
        """
        根據 agent local + global 和目標的 global 計算出目標的 local position
        """
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
