from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from safmc_msgs.msg import DropZoneInfo, SupplyZoneInfo
from common.logger import Logger
from common.qos import cmd_qos_profile
from mediator.constants import NUM_DRONES


SCHE = {
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
    _agent_task = {1: "task1", 2: "task1", 3: "task1", 4: "task1"}

    # 是否再 hotstop 等待隊友
    _is_drone_waiting_at_hotspot = [False] * (NUM_DRONES + 1)

    # Publishers
    _cmd_takeoff_pubs = [None] * (NUM_DRONES + 1)
    _cmd_drop_pubs = [None] * (NUM_DRONES + 1)
    _supply_zone_pubs = [None] * (NUM_DRONES + 1)
    _drop_zone_pubs = [None] * (NUM_DRONES + 1)

    def _update(self):
        for drone_id in range(1, NUM_DRONES + 1):
            task = self._agent_task[drone_id]

            assert task == "task1" or task == "task2"

            sz_msg = SupplyZoneInfo()
            sz_msg.supply_zone_id = SCHE[drone_id]["color"]  # "green"|"blue"
            sz_msg.aruco_marker_id = SCHE[drone_id][task]["supply_marker_id"]
            self._supply_zone_pubs[drone_id].publish(sz_msg)

            dz_msg = DropZoneInfo()
            dz_msg.drop_zone_id = SCHE[drone_id][task]["drop_zone"]  # "A"|"B"|"C"|"D"
            dz_msg.aruco_marker_id = SCHE[drone_id][task]["drop_marker_id"]
            self._drop_zone_pubs[drone_id].publish(dz_msg)

    def __init__(self):
        super().__init__("mediator")
        Logger(self)
        self.timer = self.create_timer(0.1, self._update)  # TODO[lnfu] refactor

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

            # schedule
            self._supply_zone_pubs[drone_id] = self.create_publisher(
                SupplyZoneInfo, prefix + "supply_zone", cmd_qos_profile
            )

            self._drop_zone_pubs[drone_id] = self.create_publisher(
                DropZoneInfo, prefix + "drop_zone", cmd_qos_profile
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

    def _set_is_drone_online(self, msg: Bool, drone_id: int):
        """收到上線通知, 回傳起飛指令"""
        if msg.data:
            Logger.info(f"DRONE {drone_id} IS ONLINE!")

            # 發送 TAKEOFF
            msg = Bool()
            msg.data = True
            self._cmd_takeoff_pubs[drone_id].publish(msg)

    def _set_is_drone_waiting_at_hotspot(self, msg: Bool, drone_id: int):

        # 標記該無人機已到達 hotspot
        self._is_drone_waiting_at_hotspot[drone_id] = msg.data

        if not self._is_drone_waiting_at_hotspot[drone_id]:
            return

        # 檢查隊友是否到達
        teammate_id = SCHE[drone_id]["teammate"]
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

        teammate_id = SCHE[drone_id]["teammate"]

        self._is_drone_waiting_at_hotspot[drone_id] = False
        self._is_drone_waiting_at_hotspot[teammate_id] = False

        self._agent_task[drone_id] = "task2"
        self._agent_task[teammate_id] = "task2"


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
