from agent.agent_machine import AgentMachine
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi
from agent.api.drone_api import NEDCoordinate
from agent.constants import NAV_THRESH, TAKEOFF_HEIGHT
from agent.common.context import Context

from .behavior import Behavior


class TakeoffBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api

        takeoff_coord = drone_api.start_position
        takeoff_coord = NEDCoordinate(
            takeoff_coord.x,
            takeoff_coord.y,
            takeoff_coord.z - TAKEOFF_HEIGHT
        )

        if (drone_api.goal_arrived(takeoff_coord, NAV_THRESH)):
            drone_api.set_altitude_reached(True)
        else:
            drone_api.publish_goto_setpoint(
                context.current_timestamp(), takeoff_coord)

    @staticmethod
    def proceed(context: Context, agent_machine: AgentMachine):
        drone_api: DroneApi = context.drone_api
        if drone_api.is_altitude_reached:
            if drone_api.is_loaded:
                agent_machine.walk_to_hotspot()
            else:
                agent_machine.walk_to_supply()
