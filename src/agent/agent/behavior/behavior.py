from abc import ABC, abstractmethod
from typing import Optional

from common.context import Context


class Behavior(ABC):
    @staticmethod
    @abstractmethod
    def execute(context: Context):
        pass

    @staticmethod
    @abstractmethod
    def proceed(context: Context) -> Optional[str]:
        pass
