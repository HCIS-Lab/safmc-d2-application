from .behavior import Behavior
from agent.api import DroneApi, MediatorApi
from agent.api.drone_api import NEDCoordinate
from agent.constants import NAV_THRESH

class WalkToSupplyBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi):
        supply_coord = drone_api.get_supply_coord()

        if drone_api.goal_arrived(supply_coord, NAV_THRESH):
            drone_api.set_supply_reached(True)
        else:
            drone_api.publish_goto_setpoint(supply_coord)
