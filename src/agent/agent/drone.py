from rclpy.node import Node
from transitions import Machine
from dataclasses import dataclass
from typing import Optional

from .states import States, transitions

from px4_msgs.msg import (

    OffboardControlMode,
    GotoSetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    TrajectorySetpoint

)

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, \
    QoSDurabilityPolicy

@dataclass
class NEDCoordinate:

    x: float
    y: float
    z: float

    def __str__(self):
        return f"NED(x={self.x}, y={self.y}, z={self.z})"

class Drone(Machine):
    def __init__(self, node: Node):
        super().__init__(self, states=States,
                         transitions=transitions, initial=States.IDLE)
        
        # qos_policy (Copied from autositter repo, might not fit this project)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # subscribers

        self.vehicle_local_position_sub = node.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_local_position",
            self.__set_vehicle_local_position,
            qos_profile)
        
        self.vehicle_status_sub = node.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.__set_vehicle_status,
            qos_profile
        )

        # publishers

        self.vehicle_command_pub = node.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos_profile
        )

        self.off_board_control_mode_pub = node.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
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
        
        self.__node = node

        self.__armed = False
        self.__home_coord = NEDCoordinate(0.0,0.0,0.0)
        
        self.__vehicle_local_position = VehicleLocalPosition()
        self.__local_coord = NEDCoordinate(0.0,0.0,0.0)

        self.__vehicle_status = VehicleStatus()
        
    # callback(setter) functions    
    
    def __set_vehicle_local_position(self, vehicle_local_position_msg : VehicleLocalPosition) -> None:
        self.__vehicle_local_position = vehicle_local_position_msg
        self.__local_coord.x = vehicle_local_position_msg.x
        self.__local_coord.y = vehicle_local_position_msg.y
        self.__local_coord.z = vehicle_local_position_msg.z

    def __set_vehicle_status(self, vehicle_status_msg : VehicleStatus) -> None:
        self.__vehicle_status = vehicle_status_msg

    def set_armed_status(self, armed : bool) -> None:
        self.__armed = armed
    
    def set_home_coord(self, coord : VehicleLocalPosition) -> None:
        self.__home_coord.x = coord.x
        self.__home_coord.y = coord.y
        self.__home_coord.z = coord.z


    # publish functions

    def publish_vehicle_command(self, command, **params) -> None:
        
        vehicle_command_msg = VehicleCommand()

        vehicle_command_msg.command = command   

        vehicle_command_msg.param1 = params.get('param1', 0.0)
        vehicle_command_msg.param2 = params.get('param2', 0.0)
        vehicle_command_msg.param3 = params.get('param3', 0.0)
        vehicle_command_msg.param4 = params.get('param4', 0.0)
        vehicle_command_msg.param5 = params.get('param5', 0.0)
        vehicle_command_msg.param6 = params.get('param6', 0.0)
        vehicle_command_msg.param7 = params.get('param7', 0.0)

        vehicle_command_msg.target_system = 1
        vehicle_command_msg.target_component = 1
        vehicle_command_msg.source_system = 1
        vehicle_command_msg.source_component = 1
        vehicle_command_msg.from_external = True
        vehicle_command_msg.timestamp = int(self.__node.get_clock().now().nanoseconds / 1000)

        self.vehicle_command_pub.publish(vehicle_command_msg)

    def publish_goto_setpoint(self,
                              coord : NEDCoordinate,
                              heading : Optional[float] = None,
                              max_horizontal_speed : Optional[float] = None,
                              max_vertical_speed : Optional[float] = None,
                              max_heading_rate : Optional[float] = None) -> None:
        
        goto_setpoint_msg = GotoSetpoint()
        goto_setpoint_msg.timestamp = int(self.__node.get_clock().now().nanoseconds / 1000)
        
        goto_setpoint_msg.coord.x = coord.x
        goto_setpoint_msg.coord.y = coord.y
        goto_setpoint_msg.coord.z = coord.z

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
            goto_setpoint_msg.flag_set_heading_rate = False
            goto_setpoint_msg.max_heading_rate = 0.0
        else:
            goto_setpoint_msg.flag_set_heading_rate = False
            goto_setpoint_msg.max_heading_rate = max_heading_rate

        self.goto_setpoint_pub.publish(goto_setpoint_msg)
        
        self.__node.get_logger().info(f"Publishing goto setpoint: {coord}")

        if heading is not None:
            self.__node.get_logger().info(f"With heading: {heading} rad")

        if max_horizontal_speed is not None:
            self.__node.get_logger().info(
                f"With max horizontal speed: {max_horizontal_speed} m/s")
            
        if max_vertical_speed is not None:
            self.__node.get_logger().info(
                f"With max vertical speed: {max_vertical_speed} m/s")
            
        if max_heading_rate is not None:
            self.__node.get_logger().info(
                f"With max heading rate: {max_heading_rate} rad/s")

    def publish_offboard_control_heartbeat(self, control_mode : str) -> None:

        offboard_control_mode_msg                    = OffboardControlMode()
        offboard_control_mode_msg.timestamp          = int(self.__node.get_clock().now().nanoseconds / 1000)
        offboard_control_mode_msg.position           = (control_mode == "position")
        offboard_control_mode_msg.velocity           = (control_mode == "velocity")
        offboard_control_mode_msg.acceleration       = (control_mode == "acceleration")
        offboard_control_mode_msg.attitude           = (control_mode == "attitude")
        offboard_control_mode_msg.body_rate          = (control_mode == "body_rate")
        offboard_control_mode_msg.thrust_and_torque  = (control_mode == "thrust_and_torque")
        offboard_control_mode_msg.direct_actuator    = (control_mode == "direct_actuator")

        if control_mode in ['position', 'velocity', 'acceleration', 'attitude', 'body_rate']:
            self.__node.get_logger().info(f"Control mode set to: {control_mode}")
        else:
            self.__node.get_logger().warn(
                f"Invalid control mode: {control_mode}. No changes made.")

        self.off_board_control_mode_pub.publish(offboard_control_mode_msg)

    # getter functions

    def get_home_coord(self) -> NEDCoordinate:
        return self.__home_coord
    
    def get_local_coord(self) -> NEDCoordinate:
        return self.__local_coord
    
    def get_vehicle_local_position(self) -> VehicleLocalPosition:
        return self.__vehicle_local_position

    def get_vehicle_status(self) -> VehicleStatus:
        return self.__vehicle_status

    def get_armed_status(self) -> bool:
        return self.__armed
    
    # utility functions

    def engage_offboard_mode(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0)
        self.__node.get_logger().info("Switch to offboard mode")
    
    def arm(self) -> None:
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0)
        self.__node.get_logger().info('Sending arm command')

        
