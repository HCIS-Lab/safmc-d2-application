from typing import Optional

from agent.constants import DELTA_TIME, NAV_THRESHOLD
from api import DroneApi, LidarApi
from common.apf_navigator import ApfNavigator
from common.logger import Logger
from common.ned_coordinate import NEDCoordinate

from .behavior import Behavior


class WalkToHotspotBehavior(Behavior):

    def __init__(self, logger: Logger, drone_api: DroneApi, lidar_api: LidarApi):
        super().__init__(logger)
        self.drone_api = drone_api
        self.lidar_api = lidar_api
        self.speed: float = 0.5

        self.apf_navigator = ApfNavigator(
            k_att=0.5,
            k_rep=3.0,
            safe_dist=1.5,
            max_speed=self.speed
        )


    def on_enter(self):
        # TODO hotspot 位置如何決定? 應該是要 mediator 告訴他?
        self.target_position: NEDCoordinate = NEDCoordinate(13, 6.2135, self.drone_api.local_position.z)


    def execute(self):
        obstacle_points = self.lidar_api.get_obstacle_points_2d(max_distance=5.0)
        current_location = self.drone_api.local_position
        heading = self.drone_api.heading
        vel = self.apf_navigator.compute_velocity(
            current_position=current_location,
            target_position=self.target_position,
            obstacle_points=obstacle_points,
            heading=heading
        )

        self.logger.info(f"target position: {self.target_position}, current position: {current_location}, vel: {vel}")

        # 往 target_position 移動, 速度大小是 self.speed
        # dist = NEDCoordinate.distance(current_location, self.target_position)
        # vel = (self.target_position - current_location).normalized * min(self.speed, dist)
        # self.drone_api.move_with_velocity(vel, DELTA_TIME)

        self.drone_api.move_with_velocity(vel)


    def get_next_state(self) -> Optional[str]:
        if self.drone_api.get_aruco_info_sub()[0] >= 0: # 如果畫面中有出現 aruco marker
            return "align_to_hotspot" # 開始精準定位
        return None
    

class AlignToHotspotBehavior(Behavior): # 精準定位

    def __init__(self, logger: Logger, drone_api: DroneApi): # TODO: 需要 aruco 的 node
        super().__init__(logger)
        self.drone_api = drone_api
        self.speed: float = 0.75 # 機器速度倍率
        self.dist_threshold = 0.1 # 距離誤差閾值
        self.outofframe_streak = 0 # 例外狀況：可能要退回之前的 state


    def on_enter(self):
        pass

    def execute(self):
        # 目前想法是依照 Aruco node 的資訊做 move with velocity
        # Aruco node 的回傳是無人機要移動到 Aruco marker 的距離
        aruco_info = self.drone_api.get_aruco_info_sub()

        vel_tuple = [(self.vel_factor * coord) if coord >= self.dist_threshold else 0 for coord in aruco_info[1:]]
        vel = NEDCoordinate(vel_tuple[0], vel_tuple[1], 0)
        self.drone_api.move_with_velocity(vel)


    def get_next_state(self) -> Optional[str]:
        aruco_info = self.drone_api.get_aruco_info_sub()
        if (aruco_info[0] >= 0 and                                            # aruco_info[0] := aruco marker id; -1 = not found
            all ([dist <= self.dist_threshold for dist in aruco_info[1:]])) : # 如果定位已經完成了，aruco node 傳回的各方向距離都會低於閾值
            return "wait" # 等待
        if aruco_info[0] < 0: # 偵測不到 aruco marker
            self.outofframe_streak += 1
            if self.outofframe_streak > 60: # 應該就是被神秘力量拉走了
                return "walk_to_hotspot" # 回去重走
        else:
            self.outofframe_streak = 0
        return None
