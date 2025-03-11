# TODO 使用 VehicleCommandAck 來追蹤是否設定成功 (error log)
from typing import Optional

import numpy as np
from rclpy.clock import Clock
from rclpy.node import Node

from common.coordinate import Coordinate
from common.qos import cmd_qos_profile
from px4_msgs.msg import (
    GotoSetpoint,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from .api import Api


class Px4Api(Api):

    _is_armed: bool = False
    _vehicle_timestamp: int = -1  # microseconds
    _is_each_pre_flight_check_passed: bool = False
    _heading: Optional[float] = None
    _local_position: Optional[Coordinate] = None
    _local_velocity: Optional[Coordinate] = None
    _last_state = "walk_to_supply"  # TODO 留下/不留下?
    _control_field = "position"  # TODO

    def __init__(self, node: Node):
        # Initial Values
        self._clock: Clock = node.get_clock()

        topic_prefix = f"fmu/"

        # Subscriptions
        node.create_subscription(
            VehicleLocalPosition,
            topic_prefix + "out/vehicle_local_position",
            self._set_vehicle_local_position,
            cmd_qos_profile,
        )
        node.create_subscription(
            VehicleStatus,
            topic_prefix + "out/vehicle_status",
            self._set_vehicle_status,
            cmd_qos_profile,
        )

        # Publishers
        self.__vehicle_command_pub = node.create_publisher(
            VehicleCommand, topic_prefix + "in/vehicle_command", cmd_qos_profile
        )
        self.__offboard_control_mode_pub = node.create_publisher(
            OffboardControlMode,
            topic_prefix + "in/offboard_control_mode",
            cmd_qos_profile,
        )
        self.__goto_setpoint_pub = node.create_publisher(
            GotoSetpoint, topic_prefix + "in/goto_setpoint", cmd_qos_profile
        )
        self.__trajectory_setpoint_pub = node.create_publisher(
            TrajectorySetpoint, topic_prefix + "in/trajectory_setpoint", cmd_qos_profile
        )

    @property
    def is_each_pre_flight_check_passed(self) -> bool:
        return self._is_each_pre_flight_check_passed

    @property
    def vehicle_timestamp(self) -> int:  # microseconds
        return self._vehicle_timestamp

    @property
    def is_armed(self) -> bool:
        return self._is_armed

    @property
    def last_state(self) -> str:
        return self._last_state

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def local_position(self) -> Coordinate:
        return self._local_position

    @property
    def local_velocity(self) -> Coordinate:
        return self._local_velocity

    # TODO: 留下/不留?
    def set_resume_state(self, state_name: str) -> None:
        """
        Record state under unexpected disarm circumstances
        """
        self._last_state = state_name

    def __get_timestamp(self) -> int:  # microseconds
        return int(self._clock.now().nanoseconds / 1000)

    # TODO
    def change_control_field(self, field: str) -> None:
        if field != "position" and field != "velocity":
            # ERROR
            return
        self._control_field = field

    def set_offboard_control_mode(self) -> None:
        """
        Set the offboard control mode for the drone.

        ref: https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages
        """
        offboard_control_mode_msg = OffboardControlMode()
        offboard_control_mode_msg.timestamp = self.__get_timestamp()

        for attr in [
            "position",
            "velocity",
            "acceleration",
            "attitude",
            "body_rate",
            "thrust_and_torque",
            "direct_actuator",
        ]:
            setattr(offboard_control_mode_msg, attr, False)
        setattr(offboard_control_mode_msg, self._control_field, True)
        # offboard_control_mode_msg.position = True  # TrajectorySetpoint
        # offboard_control_mode_msg.velocity = True  # TrajectorySetpoint
        self.__offboard_control_mode_pub.publish(offboard_control_mode_msg)

    def activate_offboard_control_mode(self) -> None:
        """
        Activates the offboard control mode for the drone.

        This method sends a `VehicleCommand` message to switch the drone to offboard mode
        by setting the appropriate control mode flags. The mode is switched by using the
        `VEHICLE_CMD_DO_SET_MODE` command.
        """
        vehicle_command_msg = self._get_default_vehicle_command_msg(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6
        )

        self.__vehicle_command_pub.publish(vehicle_command_msg)

    def move_to(self, position: Coordinate) -> None:
        goto_setpoint_msg = GotoSetpoint()
        goto_setpoint_msg.timestamp = self.__get_timestamp()

        goto_setpoint_msg.position = [position.x, position.y, position.z]

        # 角度不變
        goto_setpoint_msg.flag_control_heading = True
        goto_setpoint_msg.heading = 0.0

        # 不控制的參數
        goto_setpoint_msg.flag_set_max_horizontal_speed = False
        goto_setpoint_msg.flag_set_max_vertical_speed = False
        goto_setpoint_msg.flag_set_max_heading_rate = False

        self.__goto_setpoint_pub.publish(goto_setpoint_msg)

    def move_with_velocity(self, velocity: Coordinate) -> None:
        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = self.__get_timestamp()

        # 控制速度
        trajectory_setpoint_msg.velocity = [velocity.x, velocity.y, velocity.z]

        # 控制角度與角速度 (不變)
        trajectory_setpoint_msg.yaw = float(0)
        trajectory_setpoint_msg.yawspeed = float(0)

        # 其他都不控制
        for attr in ["position", "acceleration", "jerk"]:
            setattr(trajectory_setpoint_msg, attr, [np.nan] * 3)

        self.__trajectory_setpoint_pub.publish(trajectory_setpoint_msg)

    def move_with_velocity_2d(self, velocity: Coordinate) -> None:
        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = self.__get_timestamp()

        # 控制速度
        trajectory_setpoint_msg.velocity = [velocity.x, velocity.y, 0.0]

        # 控制角度與角速度 (不變)
        trajectory_setpoint_msg.yaw = float(0)
        trajectory_setpoint_msg.yawspeed = float(0)

        # 其他都不控制
        for attr in ["position", "acceleration", "jerk"]:
            setattr(trajectory_setpoint_msg, attr, [np.nan] * 3)

        self.__trajectory_setpoint_pub.publish(trajectory_setpoint_msg)

    def _set_vehicle_status(self, vehicle_status_msg: VehicleStatus) -> None:
        self._is_each_pre_flight_check_passed = (
            vehicle_status_msg.pre_flight_checks_pass
        )
        self._vehicle_timestamp = vehicle_status_msg.timestamp
        self._is_armed = (
            vehicle_status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def _set_vehicle_local_position(
        self, vehicle_local_position_msg: VehicleLocalPosition
    ) -> None:
        self._heading = vehicle_local_position_msg.heading
        self._local_position = Coordinate(
            x=vehicle_local_position_msg.x,
            y=vehicle_local_position_msg.y,
            z=vehicle_local_position_msg.z,
        )
        self._local_velocity = Coordinate(
            x=vehicle_local_position_msg.vx,
            y=vehicle_local_position_msg.vy,
            z=vehicle_local_position_msg.vz,
        )

    def _get_default_vehicle_command_msg(
        self, command, *params: float, **kwargs
    ) -> VehicleCommand:
        """
        Generate the vehicle command.\n
        defaults:\n
            params[0:7] = 0
            target_system = self.drone_id - 1\n
            target_component = 1\n
            source_system = self.drone_id - 1\n
            source_component = 1\n
            from_external = True\n
            timestamp = int(timestamp / 1000)
        """
        vehicle_command_msg = VehicleCommand()
        vehicle_command_msg.timestamp = self.__get_timestamp()

        # command
        vehicle_command_msg.command = command

        # params
        params = list(params) + [0] * (7 - len(params))
        for i, param in enumerate(params[:7], start=1):
            setattr(vehicle_command_msg, f"param{i}", float(param))

        # defaults
        vehicle_command_msg.target_system = 0
        vehicle_command_msg.target_component = 0  # all components
        vehicle_command_msg.source_system = 0
        vehicle_command_msg.source_component = 0  # all components
        vehicle_command_msg.from_external = True

        # other kwargs
        for attr, value in kwargs.items():
            try:
                setattr(vehicle_command_msg, attr, value)
            except Exception as e:
                print(e)

        return vehicle_command_msg
