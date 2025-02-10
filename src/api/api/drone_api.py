# TODO 使用 VehicleCommandAck 來追蹤是否設定成功 (error log)

from typing import Optional

from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import Twist

from common.decorators import deprecated
from common.ned_coordinate import NEDCoordinate
from px4_msgs.msg import (GotoSetpoint, OffboardControlMode,
                          TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

from .api import Api


class DroneApi(Api):
    def __init__(self, node: Node):

        self.__clock: Clock = node.get_clock()

        # Initial Values
        self.__is_armed = False
        self.__vehicle_timestamp = -1
        self.__is_each_pre_flight_check_passed = False
        self.__start_position = NEDCoordinate(0, 0, 0)

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

        self.aruco_align_velocity_sub = node.create_subscription(
            Twist,
            '/cmd_vel',
            self.__twist_callback,
            10
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

        self.trajectory_setpoint_pub = node.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos_profile
        )

    @property
    def is_each_pre_flight_check_passed(self) -> bool:
        return self.__is_each_pre_flight_check_passed

    @property
    def vehicle_timestamp(self) -> int:
        return self.__vehicle_timestamp  # microseconds

    @property
    def is_armed(self) -> bool:
        return self.__is_armed

    def __set_vehicle_status(self, vehicle_status_msg: VehicleStatus) -> None:
        self.__is_each_pre_flight_check_passed = vehicle_status_msg.pre_flight_checks_pass
        self.__vehicle_timestamp = vehicle_status_msg.timestamp
        self.__is_armed = (
            vehicle_status_msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def reset_origin(self, origin: NEDCoordinate):
        self.__origin = origin

    def __twist_callback(self, twist_msg):
        self.__twists_vector = (twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z)

    def get_twist(self):
        if hasattr(self, '_DroneApi__twists_vector'):
            return self.__twists_vector
        else:
            print('twist vector not found, return zero vector instead')
            return (0, 0, 0)
            



    @property
    def local_position(self) -> NEDCoordinate:
        return self.__local_position - self.__origin

    def __set_vehicle_local_position(self, vehicle_local_position_msg: VehicleLocalPosition):
        self.__local_position = NEDCoordinate(
            x=vehicle_local_position_msg.x,
            y=vehicle_local_position_msg.y,
            z=vehicle_local_position_msg.z
        )

    def __get_default_vehicle_command_msg(self, command, *params: float, **kwargs):
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
        vehicle_command_msg.timestamp = self.__get_timestamp()

        # command
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

        # other kwargs
        for attr, value in kwargs.items():
            try:
                setattr(vehicle_command_msg, attr, value)
            except Exception as e:
                print(e)

        return vehicle_command_msg

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

        self.vehicle_command_pub.publish(vehicle_command_msg)

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

        self.vehicle_command_pub.publish(vehicle_command_msg)

    def reset_start_position(self) -> None:
        """
        Resets the starting position to the current local position.
        """
        self.__start_position = self.local_position

    @property
    def start_position(self) -> NEDCoordinate:
        """
        The position of the drone at the moment of takeoff.
        """
        return self.__start_position

    def __get_timestamp(self) -> int:
        return int(self.__clock.now().nanoseconds / 1000)  # microseconds

    def set_offboard_control_mode(self) -> None:
        """
        Set the offboard control mode for the drone.

        position = True
        velocity = True
        timestamp = current timestamp

        ref: https://docs.px4.io/main/en/flight_modes/offboard.html#ros-2-messages
        """
        offboard_control_mode_msg = OffboardControlMode()
        offboard_control_mode_msg.timestamp = self.__get_timestamp()
        offboard_control_mode_msg.position = True  # TrajectorySetpoint
        self.offboard_control_mode_pub.publish(offboard_control_mode_msg)

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

        self.vehicle_command_pub.publish(vehicle_command_msg)

    def move_to(self, position: NEDCoordinate):

        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = self.__get_timestamp()

        trajectory_setpoint_msg.position[0] = position.x
        trajectory_setpoint_msg.position[1] = position.y
        trajectory_setpoint_msg.position[2] = position.z

        self.trajectory_setpoint_pub.publish(trajectory_setpoint_msg)

    def move_with_velocity(self, velocity: NEDCoordinate, delta_time: float):
        trajectory_setpoint_msg = TrajectorySetpoint()
        trajectory_setpoint_msg.timestamp = self.__get_timestamp()

        trajectory_setpoint_msg.velocity[0] = velocity.x
        trajectory_setpoint_msg.velocity[1] = velocity.y
        trajectory_setpoint_msg.velocity[2] = velocity.z

        trajectory_setpoint_msg.position[0] = self.local_position.x + \
            delta_time * velocity.x
        trajectory_setpoint_msg.position[1] = self.local_position.y + \
            delta_time * velocity.y
        trajectory_setpoint_msg.position[2] = self.local_position.z + \
            delta_time * velocity.z

        self.trajectory_setpoint_pub.publish(trajectory_setpoint_msg)

    @deprecated
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
