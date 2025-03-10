import math

import numpy as np

from common.coordinate import Coordinate


class BugNavigator:

    def __init__(self, safe_dist: float):
        self.safe_dist = safe_dist
        self.reverse = False

    def compute_pos(
        self,
        current_position: Coordinate,
        target_position: Coordinate,
        obstacle_points: list[tuple[float, float]],
    ):

        future_vel = self.move_in_direction_vel(current_position, target_position)
        valid = 1

        obstacle_points = list(
            filter(
                lambda obstacle: math.sqrt(
                    (
                        (obstacle[0] - future_vel.x) ** 2
                        + (-obstacle[1] - future_vel.y) ** 2
                    )
                )
                < self.safe_dist,
                obstacle_points,
            )
        )

        """
        for obstacle in obstacle_points:
            if math.sqrt(((obstacle[0] - future_vel.x)**2 + (-obstacle[1] - future_vel.y)**2)) < self.safe_dist:
                valid = 0
                break
        """

        if len(obstacle_points) == 0:
            return future_vel * (0.6)
        else:
            return self.compute_tangent_pos(current_position, obstacle_points)

    def compute_tangent_pos(
        self, current_position: Coordinate, obstacle_points: list[tuple[float, float]]
    ):

        if current_position.y > 10.0:
            self.reverse = True

        ind = np.argmin(
            [obstacle[0] ** 2 + obstacle[1] ** 2 for obstacle in obstacle_points]
        )
        nearest_point = np.array(obstacle_points[ind])
        nearest_point[1] = -nearest_point[1]
        safe_point = (
            nearest_point
            - nearest_point / np.linalg.norm(nearest_point) * self.safe_dist
        )
        tangent_vec = np.array([-nearest_point[1], nearest_point[0]])
        dest = (
            safe_point
            + (-1 if self.reverse else 1)
            * tangent_vec
            / np.linalg.norm(tangent_vec)
            * 0.3
        )
        return Coordinate(dest[0], dest[1], 0.0)

    def move_in_direction_vel(
        self, current_position: Coordinate, target_position: Coordinate
    ):
        cur_tar_vec = target_position - current_position
        return cur_tar_vec.normalized
