from abc import ABC, abstractmethod
from typing import Optional

from common.logger import Logger


class Behavior(ABC):

    def __init__(self, logger: Logger):
        self.logger = logger

    @abstractmethod
    def execute(self):
        pass

    @abstractmethod
    def get_next_state(self) -> Optional[str]:
        pass

    def log_position(self, target_position, current_position):
        """
        Log the target and current position.
        """
        self.logger.info(f"Target position: {target_position}")
        self.logger.info(f"Current position: {current_position}")
