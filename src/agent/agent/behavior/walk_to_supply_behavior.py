from agent_machine import AgentMachine
from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api import DroneApi, MediatorApi
from agent.api.drone_api import NEDCoordinate
from agent.constants import NAV_THRESH

from .behavior import Behavior


class WalkToSupplyBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        supply_coord = drone_api.get_supply_coord()

        if drone_api.goal_arrived(supply_coord, NAV_THRESH):
            drone_api.set_supply_reached(True)
        else:
            drone_api.publish_goto_setpoint(
                clock.now().nanoseconds, supply_coord)

    @staticmethod
    def proceed(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock, agent_machine: AgentMachine):
        if drone_api.get_supply_reached():
            agent_machine.load()
