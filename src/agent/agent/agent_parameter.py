from rclpy.node import Node


class AgentParameter:
    def __init__(self, node: Node):

        self._node = node
        node.declare_parameter("delta_time", 0.1)
        node.declare_parameter("takeoff_height", 0.9)
        node.declare_parameter("load_height", 0.15)
        node.declare_parameter("navigation_goal_tolerance", 0.1)
        node.declare_parameter("navigation_height_tolerance", 0.05)
        node.declare_parameter("navigation_aruco_tolerance", 0.015)
        node.declare_parameter(
            "camera_calibration_file_path",
            "/workspace/safmc-d2-bridge/camera/imx708_wide__base_soc_i2c0mux_i2c_1_imx708_1a_800x600.yaml",
        )

    @property
    def delta_time(self) -> float:
        """
        每次更新間隔 (timer update)

        單位: second
        """
        return self._node.get_parameter("delta_time").value

    @property
    def takeoff_height(self) -> float:
        """
        起飛終點高度

        單位: meter
        """
        return self._node.get_parameter("takeoff_height").value

    @property
    def load_height(self) -> float:
        """撿取 Payload 下降終點高度 (TODO[lnfu] 可能不用, 直接 landing 就好)"""
        return self._node.get_parameter("load_height").value

    @property
    def navigation_goal_tolerance(self) -> float:
        """移動過程到達目標範圍 TODO[lnfu] 註解 docstring 重寫"""
        return self._node.get_parameter("navigation_goal_tolerance").value

    @property
    def navigation_height_tolerance(self) -> float:
        """起飛高度範圍 TODO[lnfu] maybe remove"""
        return self._node.get_parameter("navigation_height_tolerance").value

    @property
    def navigation_aruco_tolerance(self) -> float:
        """aruco alignment 範圍 TODO[lnfu] rename & 註解 docstring 重寫"""
        return self._node.get_parameter("navigation_aruco_tolerance").value

    @property
    def camera_calibration_file_path(self) -> str:
        """相機校正參數檔案路徑"""
        return self._node.get_parameter("camera_calibration_file_path").value
