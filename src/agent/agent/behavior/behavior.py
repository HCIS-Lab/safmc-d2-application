from abc import ABC, abstractmethod
from typing import Optional


class Behavior(ABC):

    @abstractmethod
    def execute(self):
        pass

    @abstractmethod
    def get_next_state(self) -> Optional[str]:
        pass
