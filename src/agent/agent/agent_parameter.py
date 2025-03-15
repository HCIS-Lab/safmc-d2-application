from rclpy.node import Node


class AgentParameter:
    def __init__(self, node: Node):

        self._node = node
        node.declare_parameter("delta_time", 0.1)

        node.declare_parameter("walk_speed", 0.3)
        node.declare_parameter("align_speed", 0.3)

        node.declare_parameter("takeoff_height", 0.9)

        node.declare_parameter("walk_goal_radius", 0.1)
        node.declare_parameter("align_goal_radius", 0.015)

        node.declare_parameter("align_timeout", 3e9)

    @property
    def delta_time(self) -> float:
        """
        每次更新間隔 (timer update)

        單位: second
        """
        return self._node.get_parameter("delta_time").value

    @property
    def speed(self) -> float:
        """
        飛行速度

        單位: meter/second
        """
        return self._node.get_parameter("speed").value

    @property
    def takeoff_height(self) -> float:
        """
        起飛終點高度

        單位: meter
        """
        return self._node.get_parameter("takeoff_height").value

    @property
    def walk_goal_radius(self) -> float:
        """移動過程到達目標範圍 TODO[lnfu] 註解 docstring 重寫"""
        return self._node.get_parameter("walk_goal_radius").value

    @property
    def align_goal_radius(self) -> float:
        """aruco alignment 範圍 TODO[lnfu] rename & 註解 docstring 重寫"""
        return self._node.get_parameter("align_goal_radius").value

    @property
    def camera_calibration_file_path(self) -> str:
        """相機校正參數檔案路徑"""
        return self._node.get_parameter("camera_calibration_file_path").value

    @property
    def align_speed(self) -> float:
        return self._node.get_parameter("align_speed").value

    @property
    def align_timeout(self) -> int:
        return self._node.get_parameter("align_timeout").value
