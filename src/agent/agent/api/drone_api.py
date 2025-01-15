from typing import Optional

from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from agent.common.decorators import deprecated
from agent.common.ned_coordinate import NEDCoordinate
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

from .api import Api


class DroneApi(Api):
    def __init__(self, node: Node):

        # Initial Values
        self.__is_armed = False
        self.__vehicle_timestamp = -1
        self.__is_each_pre_flight_check_passed = False
        self.__hotspot_reached = False

        self.hotspot_coord = [
            NEDCoordinate(-6.2135, 5.65797, 0.6),
            NEDCoordinate(-4.98951, -4.52046, 0.6),
            NEDCoordinate(7.21815, 0.43991, 0.6)
        ]

        self.__is_altitude_reached = False
        self.__supply_reached = False
        self.__supply_coord = False

        # TODO: qos_policy (Copied from autositter repo, might not fit this project)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.vehicle_local_position_sub = node.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.__set_vehicle_local_position,
            qos_profile)

        self.vehicle_status_sub = node.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.__set_vehicle_status,
            qos_profile
        )

        # Publishers
        self.vehicle_command_pub = node.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile
        )

        self.offboard_control_mode_pub = node.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos_profile
        )

        self.goto_setpoint_pub = node.create_publisher(
            GotoSetpoint,
            "/fmu/in/goto_setpoint",
            qos_profile
        )

        self.goto_setpoint_pub = node.create_publisher(
            GotoSetpoint,
            "/fmu/in/goto_setpoint",
            qos_profile
        )

        self.goto_setpoint_pub = node.create_publisher(
            GotoSetpoint,
            '/fmu/in/goto_setpoint',
            qos_profile)

    @ property
    def is_each_pre_flight_check_passed(self) -> bool:
        return self.__is_each_pre_flight_check_passed

    @ property
    def vehicle_timestamp(self) -> int:
        return self.__vehicle_timestamp  # microseconds

    @ property
    def is_armed(self) -> bool:
        return self.__is_armed

    def __set_vehicle_status(self, vehicle_status_msg: VehicleStatus) -> None:
        self.__is_each_pre_flight_check_passed = vehicle_status_msg.pre_flight_checks_pass
        self.__vehicle_timestamp = vehicle_status_msg.timestamp
        self.__is_armed = (
            vehicle_status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    @ property
    def local_position(self) -> NEDCoordinate:
        return self.__local_position

    def __set_vehicle_local_position(self, vehicle_local_position_msg: VehicleLocalPosition):
        self.__local_position = NEDCoordinate(
            x=vehicle_local_position_msg.x,
            y=vehicle_local_position_msg.y,
            z=vehicle_local_position_msg.z
        )

    def set_altitude_reached(self, status: bool) -> None:
        self.__is_altitude_reached = status

    def get_default_vehicle_command_msg(self, command, timestamp: int, *params: float, **kwargs):
        '''
        Generate the vehicle command.\n
        defaults:\n
            params[0:7] = 0
            target_system = 1\n
            target_component = 1\n
            source_system = 1\n
            source_component = 1\n
            from_external = True\n
            timestamp = int(timestamp / 1000)
        '''
        vehicle_command_msg = VehicleCommand()
        vehicle_command_msg.command = command

        # params
        params = list(params) + [0] * (7 - len(params))
        for i, param in enumerate(params[:7], start=1):
            setattr(vehicle_command_msg, f'param{i}', float(param))

        # defaults
        vehicle_command_msg.target_system = 1
        vehicle_command_msg.target_component = 1
        vehicle_command_msg.source_system = 1
        vehicle_command_msg.source_component = 1
        vehicle_command_msg.from_external = True
        vehicle_command_msg.timestamp = int(
            timestamp / 1000)  # microseconds

        # other kwargs
        for attr, value in kwargs.items():
            try:
                setattr(vehicle_command_msg, attr, value)
            except Exception as e:
                print(e)

        return vehicle_command_msg

    def arm(self, timestamp: int) -> None:
        """
        Arms the drone for flight.

        Sends a command to the vehicle to arm it, allowing flight to proceed.
        This command uses `VEHICLE_CMD_COMPONENT_ARM_DISARM` with `param1=1` to arm the vehicle.
        """

        vehicle_command_msg = self.get_default_vehicle_command_msg(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            timestamp,
            1
        )

        self.vehicle_command_pub.publish(vehicle_command_msg)

    def disarm(self, timestamp: int) -> None:
        """
        Disarms the drone, preventing flight.

        Sends a command to the vehicle to disarm it, ensuring it cannot take off.
        This command uses `VEHICLE_CMD_COMPONENT_ARM_DISARM` with `param1=0` to disarm the vehicle.
        """
        vehicle_command_msg = self.get_default_vehicle_command_msg(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            timestamp,
            0
        )

        self.vehicle_command_pub.publish(vehicle_command_msg)

    def reset_start_position(self) -> None:
        """
        Resets the starting position to the current local position.
        """
        self.__start_position = self.local_position

    @ property
    def start_position(self) -> NEDCoordinate:
        return self.__start_position

    def maintain_offboard_control(self, timestamp: int) -> None:
        """
        Keeps the drone in offboard control mode by sending periodic updates.

        This method publishes an `OffboardControlMode` message with the current timestamp,
        ensuring the drone stays in offboard control mode by setting the position control flag.
        It must be continuously called to prevent the system from switching back to another mode.
        """
        offboard_control_mode_msg = OffboardControlMode()
        offboard_control_mode_msg.timestamp = int(
            timestamp / 1000)  # microseconds
        offboard_control_mode_msg.position = True
        self.offboard_control_mode_pub.publish(offboard_control_mode_msg)

    def activate_offboard_control_mode(self, timestamp: int) -> None:
        """
        Activates the offboard control mode for the drone.

        This method sends a `VehicleCommand` message to switch the drone to offboard mode 
        by setting the appropriate control mode flags. The mode is switched by using the
        `VEHICLE_CMD_DO_SET_MODE` command.
        """
        vehicle_command_msg = self.get_default_vehicle_command_msg(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            timestamp,
            1,
            6
        )

        self.vehicle_command_pub.publish(vehicle_command_msg)

    # TODO refactor
    def publish_goto_setpoint(self,
                              timestamp: int,
                              coord: NEDCoordinate,
                              heading: Optional[float] = None,
                              max_horizontal_speed: Optional[float] = None,
                              max_vertical_speed: Optional[float] = None,
                              max_heading_rate: Optional[float] = None) -> None:

        goto_setpoint_msg = GotoSetpoint()
        goto_setpoint_msg.timestamp = int(
            timestamp / 1000)  # microseconds

        goto_setpoint_msg.position[0] = coord.x
        goto_setpoint_msg.position[1] = coord.y
        goto_setpoint_msg.position[2] = coord.z

        if heading is None:
            goto_setpoint_msg.flag_control_heading = False
            goto_setpoint_msg.heading = 0.0
        else:
            goto_setpoint_msg.flag_control_heading = True
            goto_setpoint_msg.heading = heading

        if max_horizontal_speed is None:
            goto_setpoint_msg.flag_set_max_horizontal_speed = False
            goto_setpoint_msg.max_horizontal_speed = 0.0
        else:
            goto_setpoint_msg.flag_set_max_horizontal_speed = False
            goto_setpoint_msg.max_horizontal_speed = max_horizontal_speed

        if max_vertical_speed is None:
            goto_setpoint_msg.flag_set_max_vertical_speed = False
            goto_setpoint_msg.max_vertical_speed = 0.0
        else:
            goto_setpoint_msg.flag_set_max_vertical_speed = False
            goto_setpoint_msg.max_vertical_speed = max_vertical_speed

        if max_heading_rate is None:
            goto_setpoint_msg.flag_set_max_heading_rate = False
            goto_setpoint_msg.max_heading_rate = 0.0
        else:
            goto_setpoint_msg.flag_set_max_heading_rate = False
            goto_setpoint_msg.max_heading_rate = max_heading_rate

        self.goto_setpoint_pub.publish(goto_setpoint_msg)

    @property
    def is_altitude_reached(self) -> bool:
        return self.__is_altitude_reached

    def get_supply_reached(self) -> bool:
        return self.__supply_reached

    def get_supply_coord(self) -> NEDCoordinate:
        return self.__supply_coord

    @deprecated
    def goal_arrived(self, target: NEDCoordinate, thresh: float) -> bool:
        return (self.__local_position.x - target.x)**2 + \
            (self.__local_position.y - target.y)**2 + \
            (self.__local_position.z - target.z)**2 <= thresh

    def set_hotspot_reached(self, reached: bool) -> None:
        self.__hotspot_reached = reached

    def get_hotspot_reached(self) -> bool:
        return self.__hotspot_reached

    def get_hotspot_coord(self) -> NEDCoordinate:
        return self.hotspot_coord
