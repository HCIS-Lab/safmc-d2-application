from .behavior import Behavior
from agent.api.drone_api import DroneApi
from agent.api.mediator_api import MediatorApi
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.clock import Clock
from agent.constants import NAV_THRESH
from agent.common.context import Context
from agent_machine import AgentMachine

class ToHotspotBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
        print("ToHotspot Behavior")
        hotspot_coord = drone_api.get_hotspot_coord()
        if drone_api.goal_arrived(hotspot_coord[0], NAV_THRESH):
            drone_api.set_hotspot_reached(True)
        else:
            drone_api.publish_goto_setpoint(clock.now().nanoseconds, hotspot_coord[0])
            drone_api.set_hotspot_reached(False)
    
    @staticmethod
    def proceed(context: Context, agent_machine: AgentMachine):
        drone_api: DroneApi = context.drone_api
        if drone_api.is_altitude_reached:
            if drone_api.get_hotspot_reached():
                agent_machine.walk_to_hotspot()
            
    