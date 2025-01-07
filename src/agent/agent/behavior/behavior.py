from abc import ABC, abstractmethod
from api import DroneApi, MediatorApi


class Behavior(ABC):
    @abstractmethod
    @staticmethod
    def execute(self, drone_api: DroneApi, mediator_api: MediatorApi):
        pass
