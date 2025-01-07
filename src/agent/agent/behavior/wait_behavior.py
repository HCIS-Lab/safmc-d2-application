from .behavior import Behavior
from api import DroneApi, MediatorApi


class WaitBehavior(Behavior):
    @staticmethod
    def execute(self, drone_api: DroneApi, mediator_api: MediatorApi):
        mediator_api.wait_to_drop()
