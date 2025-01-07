from .behavior import Behavior
from api import DroneApi, MediatorApi


class WaitBehavior(Behavior):
    def execute(self, drone_api: DroneApi, mediator_api: MediatorApi):
        mediator_api.wait_to_drop()
