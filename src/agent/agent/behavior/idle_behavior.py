from typing import Optional

from api import DroneApi
from common.context import Context

from .behavior import Behavior


class IdleBehavior(Behavior):
    def execute(self, ctx: Context):

        drone_api: DroneApi = ctx.drone_api
        logger = ctx.logger  # TODO log 限流

        logger.info(f"Armed status: {drone_api.is_armed}")
        logger.info(f"Vehicle timestamp: {drone_api.vehicle_timestamp}")
        logger.info(
            f"Preflight checks passed: {drone_api.is_each_pre_flight_check_passed}")

        # TODO 改成 mediator 來控制
        if (not drone_api.is_armed) and drone_api.is_each_pre_flight_check_passed:
            logger.info("Drone is ready to arm and start offboard control.")
            drone_api.reset_start_position()
            drone_api.activate_offboard_control_mode(
                ctx.get_current_timestamp())
            drone_api.arm(ctx.get_current_timestamp())

    def get_next_state(self, ctx: Context) -> Optional[str]:
        drone_api: DroneApi = ctx.drone_api

        if drone_api.is_armed:
            return "takeoff"
        return None

    def on_exit(self, ctx: Context):
        print("TESTESTEST")
