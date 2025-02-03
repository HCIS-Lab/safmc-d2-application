# common/apf_navigator.py

import math
from common.ned_coordinate import NEDCoordinate

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
        current_position: NEDCoordinate,
        target_position: NEDCoordinate,
        obstacle_points: list[tuple[float, float]],
        heading: float
    ) -> NEDCoordinate:
        """
        Args:
            current_position: 無人機目前位置 (NED 座標)
            target_position:  目標位置 (NED 座標)
            obstacle_points:  由 LiDAR 偵測到的障礙物 (x, y)，以無人機自身為原點, 水平 2D。

        Returns:
            NEDCoordinate: (vx, vy, vz) 的速度向量 (這邊 z=0 只在水平面動).
        """
        # 1) 吸引力 F_att
        diff_3d = target_position - current_position
        f_att_x = self.k_att * diff_3d.x
        f_att_y = self.k_att * diff_3d.y

        # 2) 排斥力 F_rep (對每一點做疊加)
        f_rep_x = 0.0
        f_rep_y = 0.0
        for (obs_x, obs_y) in obstacle_points:
            obs_x = current_position.x + obs_x * math.cos(heading) - obs_y * math.sin(heading)
            obs_y = current_position.y + obs_x * math.sin(heading) + obs_y * math.cos(heading)
            dx = obs_x - current_position.x
            dy = obs_y - current_position.y
            d = math.sqrt(dx*dx + dy*dy)
            if d < self.safe_dist:
                rep_val = self.k_rep * (1.0/d - 1.0/self.safe_dist) * (1.0/d**2)
                norm_x = -dx / d
                norm_y = -dy / d

                f_rep_x += rep_val * norm_x
                f_rep_y -= rep_val * norm_y

        if len(obstacle_points) > 0:
            f_rep_x /= len(obstacle_points)
            f_rep_y /= len(obstacle_points)

        # 3) 合成速度並限幅
        fx = f_att_x + f_rep_x
        fy = f_att_y + f_rep_y
        print("Attract Force:", f_att_x , f_att_y)
        print("Repulse Force:", f_rep_x , f_rep_y)
        speed_mag = math.sqrt(fx**2 + fy**2)
        if speed_mag > self.max_speed:
            scale = self.max_speed / speed_mag
            fx *= scale
            fy *= scale

        # Z 軸暫時維持 0，不動垂直高度
        return NEDCoordinate(fx, fy, 0.0)
