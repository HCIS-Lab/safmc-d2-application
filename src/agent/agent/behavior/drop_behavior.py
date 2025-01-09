from .behavior import Behavior
from api import DroneApi, MediatorApi


class DropBehavior(Behavior):
    @staticmethod
    def execute(self, drone_api: DroneApi, mediator_api: MediatorApi):
        drone_api.drop_payload()
