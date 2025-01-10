from .behavior import Behavior
from agent.api import DroneApi, MediatorApi


class WaitBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi):
        mediator_api.wait_to_drop()
