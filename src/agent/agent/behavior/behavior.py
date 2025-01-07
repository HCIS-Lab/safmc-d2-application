from abc import ABC, abstractmethod
from api import DroneApi, MediatorApi


class Behavior(ABC):
    @staticmethod
    @abstractmethod
    def execute(self, drone_api: DroneApi, mediator_api: MediatorApi):
        pass
