from abc import ABC, abstractmethod
from typing import Optional

from common.logger import Logger


class Behavior(ABC):

    def __init__(self, logger: Logger):
        self.logger = logger

    def _execute(self):
        self.logger.info(
            f"Vehicle timestamp: {self.drone_api.vehicle_timestamp}"
        )  # TODO[lnfu]

    @abstractmethod
    def execute(self):
        pass

    @abstractmethod
    def get_next_state(self) -> Optional[str]:
        pass

    # TODO[lnfu]
    def log_position(self, target_position, current_position):
        """
        Log the target and current position.
        """
        self.logger.info(
            f"Target position: {target_position}\nCurrent position: {current_position}\n\n"
        )
