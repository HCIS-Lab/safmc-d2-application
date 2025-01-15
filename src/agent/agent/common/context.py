from typing import Optional, Type

from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from agent.api.api import Api


class Context():
    def __init__(self, logger: RcutilsLogger, clock: Clock, **apis):
        self.logger: RcutilsLogger = logger
        self.clock: Clock = clock
        for api_name, api in apis.items():
            setattr(self, api_name, api)

    # getters
    def get_api(self, api_name: str) -> Optional[Type[Api]]:
        try:
            return getattr(self, api_name)
        except AttributeError as e:
            print(f"AttributeError: {e}")
            return None

    def get_current_timestamp(self) -> int:
        return self.clock.now().nanoseconds

    # setters
    def set_api(self, new_api_name: str, new_api: Type[Api]):
        try:
            setattr(self, new_api_name, new_api)
        except Exception as e:
            print(f"Exception: {e}")
