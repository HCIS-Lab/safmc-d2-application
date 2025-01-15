from typing import Optional

from agent.api import DroneApi
from agent.common.context import Context

from .behavior import Behavior


class IdleBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        print("IdleBehavior")
        drone_api: DroneApi = context.drone_api
        if (not drone_api.is_armed) and drone_api.vehicle_timestamp > 10000000 and drone_api.is_each_pre_flight_check_passed:
            # TODO: self.drone.get_vehicle_status().nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: why?
            drone_api.reset_start_position()
            drone_api.activate_offboard_control_mode(
                context.get_current_timestamp())
            drone_api.arm(context.get_current_timestamp())

    @staticmethod
    def proceed(context: Context) -> Optional[str]:
        if context.drone_api.is_armed:
            return "takeoff"
        return None
