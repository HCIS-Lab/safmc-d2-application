from agent.agent_machine import AgentMachine
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi
from agent.api.drone_api import NEDCoordinate
from agent.constants import LOAD_HEIGHT, NAV_THRESH, TAKEOFF_HEIGHT
from agent.common.context import Context

from .behavior import Behavior


class LoadBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api

        load_coord = drone_api.local_position
        load_coord = NEDCoordinate(
            load_coord.x,
            load_coord.y,
            -LOAD_HEIGHT
        )

        if (drone_api.goal_arrived(load_coord, NAV_THRESH)):
            drone_api.activate_magnet()
            if drone_api.is_loaded:
                load_coord = drone_api.local_position
                load_coord = NEDCoordinate(
                    load_coord.x,
                    load_coord.y,
                    LOAD_HEIGHT - TAKEOFF_HEIGHT
                )

                if (drone_api.goal_arrived(load_coord, NAV_THRESH)):
                    drone_api.has_loaded()
                else:
                    drone_api.publish_goto_setpoint(
                        context.current_timestamp(), load_coord)
        else:
            drone_api.publish_goto_setpoint(
                context.current_timestamp(), load_coord)

    @staticmethod
    def proceed(context: Context, agent_machine: AgentMachine):
        if context.drone_api.is_loaded:
            agent_machine.walk_to_hotspot()
