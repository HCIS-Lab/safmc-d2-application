from agent.api import DroneApi
from agent.common.context import Context
from agent.constants import NAV_THRESH

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):
    @staticmethod
    def execute(context: Context):
        drone_api: DroneApi = context.drone_api

        supply_coord = drone_api.get_supply_coord()

        if drone_api.goal_arrived(supply_coord, NAV_THRESH):
            drone_api.set_supply_reached(True)
        else:
            drone_api.publish_goto_setpoint(
                context.get_current_timestamp(), supply_coord)

    @staticmethod
    def proceed(context: Context, agent_machine):
        if context.drone_api.get_supply_reached():
            agent_machine.load()
