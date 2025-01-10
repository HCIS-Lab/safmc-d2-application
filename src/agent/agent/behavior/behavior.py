from abc import ABC, abstractmethod
from agent.api import DroneApi, MediatorApi


class Behavior(ABC):
    @staticmethod
    @abstractmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi):
        pass
