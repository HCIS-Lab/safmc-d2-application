
import math
import numpy as np

from common.ned_coordinate import NEDCoordinate

class bugNavigator:

    def __init__(self, safe_dist : float):
        self.safe_dist = safe_dist

    def compute_pos(self, current_position : NEDCoordinate, target_position: NEDCoordinate, obstacle_points: list[tuple[float, float]]):
        
        future_vel = self.move_in_direction_vel(current_position, target_position)
        valid = 1
        for obstacle in obstacle_points:
            if math.sqrt(((obstacle[1] - future_vel.x)**2 + (obstacle[0] - future_vel.y)**2)) < self.safe_dist:
                valid = 0
                break

        if valid : return future_vel
        else : return self.compute_tangent_pos(obstacle_points)

    def compute_tangent_pos(self, obstacle_points: list[tuple[float, float]]):
        ind = np.argmin([obstacle[0]**2 + obstacle[1]**2 for obstacle in obstacle_points])
        nearest_point = np.array(obstacle_points[ind])
        print(nearest_point)
        #temp = nearest_point[0]
        #nearest_point[0] = nearest_point[1]
        #nearest_point[1] = temp
        safe_point = nearest_point - nearest_point/np.linalg.norm(nearest_point) * self.safe_dist
        tangent_vec = np.array(-nearest_point[1], nearest_point[0])
        dest = safe_point + tangent_vec/np.linalg.norm(tangent_vec)*0.1
        return NEDCoordinate(dest[0], dest[1], 0.0)

    def move_in_direction_vel(self, current_position : NEDCoordinate, target_position: NEDCoordinate):
        cur_tar_vec = target_position - current_position
        cur_tar_vec /= math.sqrt(cur_tar_vec.x**2 + cur_tar_vec.y**2)
        return NEDCoordinate(cur_tar_vec.x, cur_tar_vec.y, 0)
    