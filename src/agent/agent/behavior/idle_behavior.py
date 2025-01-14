

from .behavior import Behavior

from agent.api import DroneApi, MediatorApi
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.clock import Clock


class IdleBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        if (not drone_api.is_armed) and drone_api.vehicle_timestamp > 10000000 and drone_api.is_each_pre_flight_check_passed:
            # TODO: self.drone.get_vehicle_status().nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: why?
            drone_api.reset_start_position()
            drone_api.activate_offboard_control_mode(clock.now().nanoseconds)
            drone_api.arm(clock.now().nanoseconds)
