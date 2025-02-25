# common/apf_navigator.py

import math

from common.coordinate import Coordinate


class ApfNavigator:

    def __init__(
        self,
        k_att: float = 1.0,        # 吸引力係數
        k_rep: float = 1.5,        # 排斥力係數
        safe_dist: float = 1.0,    # 物體安全距離 (m)
        max_speed: float = 0.5     # UAV 速度上限 (m/s)
    ):
        self.k_att = k_att
        self.k_rep = k_rep
        self.safe_dist = safe_dist
        self.max_speed = max_speed

    def compute_velocity(
        self,
        current_position: Coordinate,
        target_position: Coordinate,
        obstacle_points: list[tuple[float, float]],
        mediator_points: list[tuple[float, float]],
        heading: float
    ) -> Coordinate:
        """
        Args:
            current_position: 無人機目前位置 (NED 座標)
            target_position:  目標位置 (NED 座標)
            obstacle_points:  由 LiDAR 偵測到的障礙物 (x, y)，以無人機自身為原點, 水平 2D。

        Returns:
            NEDCoordinate: (vx, vy, vz) 的速度向量 (這邊 z=0 只在水平面動).
        """

        static_obstacle = []
        for (obs_x, obs_y) in obstacle_points:
            obs_x = current_position.x + obs_x * math.cos(heading) - obs_y * math.sin(heading)
            obs_y = current_position.y + obs_x * math.sin(heading) + obs_y * math.cos(heading)
            static_obstacle.append((obs_x - current_position.x, obs_y - current_position.y))
            
        # 1) 吸引力 F_att
        diff_3d = target_position - current_position
        f_att_x = self.k_att * diff_3d.x
        f_att_y = self.k_att * diff_3d.y
        f_att_z = self.k_att * diff_3d.z
        rep_obstacle_x, rep_obstacle_y = self.compute_repulsive_force(static_obstacle)
        rep_mediator_x, rep_mediator_y = self.compute_repulsive_force(mediator_points)

        # 將兩組排斥力各自平均後，分別賦予相等重要性
        f_rep_x = rep_obstacle_x + rep_mediator_x
        f_rep_y = rep_obstacle_y + rep_mediator_y

        # 3) 合成速度並限幅
        fx = f_att_x + f_rep_x
        fy = f_att_y + f_rep_y
        fz = f_att_z
        # fz = 0
        # print("Attract Force:", f_att_x, f_att_y, f_att_z)
        # print("Repulse Force:", f_rep_x , f_rep_y, 0.0)
        speed_mag = math.sqrt(fx**2 + fy**2)
        if speed_mag > self.max_speed:
            scale = self.max_speed / speed_mag
            fx *= scale
            fy *= scale
            # fz *= scale

        return Coordinate(fx, fy, fz)
    
    def compute_repulsive_force(self, obstacle_points: list[tuple[float, float]]) -> tuple[float, float]:
        rep_x = 0.0
        rep_y = 0.0
        count = 0
        for (obs_x, obs_y) in obstacle_points:
            d = math.sqrt(obs_x**2 + obs_y**2)
            if d > 0 and d < self.safe_dist:
                rep_val = self.k_rep * (1.0/d - 1.0/self.safe_dist) * (1.0/d**2)
                norm_x = -obs_x / d
                norm_y = -obs_y / d
                rep_x += rep_val * norm_x
                rep_y -= rep_val * norm_y
                count += 1
        if count > 0:
            rep_x /= count
            rep_y /= count
        return rep_x, rep_y

