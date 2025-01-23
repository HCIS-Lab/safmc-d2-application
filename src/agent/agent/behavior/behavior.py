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

    def on_enter(self):
        """
        進入狀態時的 callback function
        """
        pass

    def on_exit(self):
        """
        離開狀態時的 callback function
        """
        pass
