from rclpy.node import Node
from common.qos import cmd_qos_profile, status_qos_profile, sensor_qos_profile
from common.logger import Logger
from .api import Api
from safmc_msgs.msg import TagPosition

from typing import Dict, List, Optional
from common.coordinate import Coordinate

NUM_DRONES = 4


class UwbApi(Api):
    _id: int

    _agent_global_p: Optional[Dict[int, Coordinate]] = {}
    _supply_zone_global_p: Optional[Dict[str, Coordinate]] = {}
    _drop_zone_global_p: Optional[Dict[str, Coordinate]] = {}

    def __init__(self, node: Node):
        node.create_subscription(
            TagPosition, "/tag_position", self._set_tag_position, sensor_qos_profile
        )
        self._id = int((node.get_namespace()).split("/px4_")[1])
        Logger.info(f"ID = {self._id}")

    def get_drop_zone_local_p(
        self, dz_code: str, agent_local_p: Coordinate
    ) -> Coordinate:
        assert dz_code == "A" or dz_code == "B" or dz_code == "C" or dz_code == "D"
        return self._get_target_local_p(
            agent_local_p, self._drop_zone_global_p[dz_code]
        )

    def _get_target_local_p(
        self,
        agent_local_p: Coordinate,
        target_global_p: Coordinate,
    ):
        """
        根據 agent local + global 和目標的 global 計算出目標的 local position
        """
        if agent_local_p is None:
            return None
        if target_global_p is None:
            return None

        return Coordinate(
            agent_local_p.x + target_global_p.x - self._agent_global_p[self._id].x,
            agent_local_p.y + target_global_p.y - self._agent_global_p[self._id].y,
            agent_local_p.z + target_global_p.z - self._agent_global_p[self._id].z,
        )

    def get_other_agent_local_ps(self, agent_local_p: Coordinate) -> List[Coordinate]:
        return [
            self._get_target_local_p(agent_local_p, self._agent_global_p[drone_id])
            for drone_id in range(1, NUM_DRONES + 1)
            if drone_id != self._id  # other drones
        ]

    def get_supply_zone_local_ps(
        self, sz_code: str, agent_local_p: Coordinate
    ) -> Optional[List[Coordinate]]:
        assert sz_code == "green" or sz_code == "blue"

        if sz_code == "green":
            sign_ = 1
        elif sz_code == "blue":
            sign_ = -1
        else:
            Logger.error("sz_code ERROR")
            return

        if sz_code not in self._supply_zone_global_p:
            Logger.info("no supply zone global position")
            return None

        Logger.info(f"agent local position = {agent_local_p}")
        Logger.info(
            f"supply zone global position = {self._supply_zone_global_p[sz_code]}"
        )

        point_1 = self._get_target_local_p(
            agent_local_p,
            self._supply_zone_global_p[sz_code] + sign_ * Coordinate.front * 3,
        )
        point_2 = self._get_target_local_p(
            agent_local_p,
            self._supply_zone_global_p[sz_code] - sign_ * Coordinate.front * 3,
        )
        point_3 = self._get_target_local_p(
            agent_local_p,
            self._supply_zone_global_p[sz_code]
            - sign_ * Coordinate.front * 3
            + Coordinate.right * 1,
        )
        point_4 = self._get_target_local_p(
            agent_local_p,
            self._supply_zone_global_p[sz_code]
            + sign_ * Coordinate.front * 3
            + Coordinate.right * 1,
        )
        return [point_1, point_2, point_3, point_4]

    def _set_tag_position(self, msg):
        """記下 UWB 傳來的資料"""
        # Logger.info(f"{msg.eui}")
        if msg.eui == "01:01":
            self._agent_global_p[1] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "02:02":
            self._agent_global_p[2] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
            # Logger.info(f"agent global position = {self._agent_global_p[2]}")
        elif msg.eui == "03:03":
            self._agent_global_p[3] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "04:04":
            self._agent_global_p[4] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "05:05":
            # Logger.info("receive blue global location")
            self._supply_zone_global_p["blue"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "06:06":
            # Logger.info("receive green global location")
            self._supply_zone_global_p["green"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "07:07":
            self._drop_zone_global_p["A"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "08:08":
            self._drop_zone_global_p["B"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "09:09":
            self._drop_zone_global_p["C"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        elif msg.eui == "10:10":
            self._drop_zone_global_p["D"] = Coordinate(x=msg.x, y=msg.y, z=msg.z)
        else:
            Logger.error("unknown eui")
