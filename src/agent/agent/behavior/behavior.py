from abc import ABC, abstractmethod
from typing import Optional

from common.context import Context


class Behavior(ABC):
    @abstractmethod
    def execute(self, context: Context):
        pass

    @abstractmethod
    def get_next_state(self, context: Context) -> Optional[str]:
        pass
