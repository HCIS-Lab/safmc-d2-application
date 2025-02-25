# TODO 使用 VehicleCommandAck 來追蹤是否設定成功 (error log)
import numpy as np
from typing import Optional

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from common.coordinate import Coordinate
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode,
                          TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

from .api import Api
from agent.constants import TAKEOFF_HEIGHT


class DroneApi(Api):
    def __init__(self, node: Node, drone_id: int):

        # Initial Values
        self.__drone_id: int = drone_id
        self.__clock: Clock = node.get_clock()

        self.__is_armed: bool = False
        self.__vehicle_timestamp: int = -1  # microseconds
        self.__is_each_pre_flight_check_passed: bool = False
        self.__heading: Optional[float] = None
        self.__local_position: Optional[Coordinate] = None
        self.__local_velocity: Optional[Coordinate] = None
        self.__last_state = "walk_to_supply"  # TODO 留下/不留下?
        self.__control_field = "position"  # TODO

        self.pid_integral_x = 0.0
        self.pid_integral_y = 0.0
        self.pid_integral_z = 0.0
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_z = 0.0
        self.pid_last_time = self.__get_timestamp()

        # TODO
        self.__start_position: Optional[Coordinate] = None

        # QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        topic_prefix: str = f"/px4_{self.__drone_id}/fmu/"

        node.create_subscription(VehicleLocalPosition,
                                 topic_prefix+"out/vehicle_local_position",
                                 self.__set_vehicle_local_position, qos_profile)

        node.create_subscription(VehicleStatus,
                                 topic_prefix+"out/vehicle_status",
                                 self.__set_vehicle_status, qos_profile)

        # Publishers
        self.__vehicle_command_pub = node.create_publisher(VehicleCommand,
                                                           topic_prefix+"in/vehicle_command",
                                                           qos_profile)

        self.__offboard_control_mode_pub = node.create_publisher(OffboardControlMode,
                                                                 topic_prefix+"in/offboard_control_mode",
                                                                 qos_profile)

        self.__goto_setpoint_pub = node.create_publisher(GotoSetpoint,
                                                         topic_prefix+"in/goto_setpoint",
                                                         qos_profile)

        self.__trajectory_setpoint_pub = node.create_publisher(TrajectorySetpoint,
                                                               topic_prefix+"in/trajectory_setpoint",
                                                               qos_profile)

    @property
    def is_each_pre_flight_check_passed(self) -> bool:
        return self.__is_each_pre_flight_check_passed

    @property
    def vehicle_timestamp(self) -> int:  # microseconds
        return self.__vehicle_timestamp

    @property
    def is_armed(self) -> bool:
        return self.__is_armed

    @property
    def last_state(self) -> str:
        return self.__last_state

    @property
    def heading(self) -> float:
        return self.__heading

    @property
    def local_position(self) -> Coordinate:
        return self.__local_position

    @property
    def local_velocity(self) -> Coordinate:
        return self.__local_velocity

    def reset_start_position(self) -> None:
        self.__start_position = self.__local_position

    def arm(self) -> None:
        """
        Arms the drone for flight.

        Sends a command to the vehicle to arm it, allowing flight to proceed.
        This command uses `VEHICLE_CMD_COMPONENT_ARM_DISARM` with `param1=1` to arm the vehicle.
        """

        vehicle_command_msg = self.__get_default_vehicle_command_msg(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1
        )

        self.__vehicle_command_pub.publish(vehicle_command_msg)

    def disarm(self) -> None:
        """
        Disarms the drone, preventing flight.

        Sends a command to the vehicle to disarm it, ensuring it cannot take off.
        This command uses `VEHICLE_CMD_COMPONENT_ARM_DISARM` with `param1=0` to disarm the vehicle.
        """
        vehicle_command_msg = self.__get_default_vehicle_command_msg(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0
        )

        self.__vehicle_command_pub.publish(vehicle_command_msg)

    # TODO: 留下/不留?
    def set_resume_state(self, state_name: str) -> None:
        """
        Record state under unexpected disarm circumstances
        """
        self.__last_state = state_name

    def __get_timestamp(self) -> int:  # microseconds
        return int(self.__clock.now().nanoseconds / 1000)

    # TODO
    def change_control_field(self, field: str) -> None:
        if field != "position" and field != "velocity":
            # ERROR
            return
        self.__control_field = field

    def set_offboard_control_mode(self) -> None:
        """
        Set the offboard control mode for the drone.

        ref: https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages
        """
        offboard_control_mode_msg = OffboardControlMode()
        offboard_control_mode_msg.timestamp = self.__get_timestamp()

        for attr in ["position", "velocity", "acceleration", "attitude", "body_rate", "thrust_and_torque", "direct_actuator"]:
            setattr(offboard_control_mode_msg, attr, False)
        setattr(offboard_control_mode_msg, self.__control_field, True)
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
        vehicle_command_msg = self.__get_default_vehicle_command_msg(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1,
            6
        )

        self.__vehicle_command_pub.publish(vehicle_command_msg)

    def compute_pid(self, target: float, current: float, dt: float, Kp: float, Ki: float, Kd: float,
                    integral: float, last_error: float):
        error = target - current
        new_integral = integral + error * dt
        derivative = (error - last_error) / dt
        output = Kp * error + Ki * new_integral + Kd * derivative
        return output, new_integral, error
    
    def move_to(self, target: Coordinate) -> None:
        current_time = self.__get_timestamp()
        dt = current_time - self.pid_last_time if (current_time - self.pid_last_time) > 0.001 else 0.01
        self.pid_last_time = current_time

        Kp_x, Ki_x, Kd_x = 0.5, 0.01, 0.1
        Kp_y, Ki_y, Kd_y = 0.5, 0.01, 0.1
        Kp_z, Ki_z, Kd_z = 1.2, 0.02, 0.1

        desired_vx, self.pid_integral_x, self.last_error_x = self.compute_pid(
            target.x, self.local_position.x, dt, Kp_x, Ki_x, Kd_x,
            self.pid_integral_x, self.last_error_x
        )

        desired_vy, self.pid_integral_y, self.last_error_y = self.compute_pid(
            target.y, self.local_position.y, dt, Kp_y, Ki_y, Kd_y,
            self.pid_integral_y, self.last_error_y
        )

        desired_vz, self.pid_integral_z, self.last_error_z = self.compute_pid(
            target.z, self.local_position.z, dt, Kp_z, Ki_z, Kd_z,
            self.pid_integral_z, self.last_error_z
        )

        max_vx = 1.0  
        max_vy = 1.0
        max_vz = 0.6  
        desired_vx = max(-max_vx, min(desired_vx, max_vx))
        desired_vy = max(-max_vy, min(desired_vy, max_vy))
        desired_vz = max(-max_vz, min(desired_vz, max_vz))

        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = current_time

        trajectory_setpoint_msg.velocity = [desired_vx, desired_vy, desired_vz]

        trajectory_setpoint_msg.yaw = float(0)
        trajectory_setpoint_msg.yawspeed = float(0)

        for attr in ["position", "acceleration", "jerk"]:
            setattr(trajectory_setpoint_msg, attr, [np.nan] * 3)

        self.__trajectory_setpoint_pub.publish(trajectory_setpoint_msg)

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

    def __set_vehicle_status(self, vehicle_status_msg: VehicleStatus) -> None:
        self.__is_each_pre_flight_check_passed = vehicle_status_msg.pre_flight_checks_pass
        self.__vehicle_timestamp = vehicle_status_msg.timestamp
        self.__is_armed = (
            vehicle_status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def __set_vehicle_local_position(self, vehicle_local_position_msg: VehicleLocalPosition) -> None:
        self.__heading = vehicle_local_position_msg.heading
        self.__local_position = Coordinate(
            x=vehicle_local_position_msg.x,
            y=vehicle_local_position_msg.y,
            z=vehicle_local_position_msg.z
        )
        self.__local_velocity = Coordinate(
            x=vehicle_local_position_msg.vx,
            y=vehicle_local_position_msg.vy,
            z=vehicle_local_position_msg.vz
        )

    def __get_default_vehicle_command_msg(self, command, *params: float, **kwargs) -> VehicleCommand:
        '''
        Generate the vehicle command.\n
        defaults:\n
            params[0:7] = 0
            target_system = self.drone_id - 1\n
            target_component = 1\n
            source_system = self.drone_id - 1\n
            source_component = 1\n
            from_external = True\n
            timestamp = int(timestamp / 1000)
        '''
        vehicle_command_msg = VehicleCommand()
        vehicle_command_msg.timestamp = self.__get_timestamp()

        # command
        vehicle_command_msg.command = command

        # params
        params = list(params) + [0] * (7 - len(params))
        for i, param in enumerate(params[:7], start=1):
            setattr(vehicle_command_msg, f'param{i}', float(param))

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
