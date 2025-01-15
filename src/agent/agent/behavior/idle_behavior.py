

from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi
from agent.common.context import Context

from .behavior import Behavior


class IdleBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api
        if (not drone_api.is_armed) and drone_api.vehicle_timestamp > 10000000 and drone_api.is_each_pre_flight_check_passed:
            # TODO: self.drone.get_vehicle_status().nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: why?
            drone_api.reset_start_position()
            drone_api.activate_offboard_control_mode(
                context.current_timestamp())
            drone_api.arm(context.current_timestamp())

    @staticmethod
    def proceed(context: Context, agent_machine):
        if context.drone_api.is_armed:
            agent_machine.takeoff()
