from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger
from api.api import Api

from typing import Type, Optional

class Context():
    def __init__(self, logger: RcutilsLogger, clock: Clock, **apis):
        self.logger = logger
        self.clock = clock
        for api_name, api in apis.items():
            setattr(self, api_name, api)
    
    # getters
    def get_api(self, api_name: str) -> Optional[Type[Api]]:
        try:
            return getattr(self, api_name)
        except AttributeError as e:
            print(f"AttributeError: {e}")
            return None
    
    def current_timestamp(self) -> int:
        return self._clock.now().nanoseconds

    # setters
    def api_setter(self, new_api_name: str, new_api: Type[Api]):
        try:
            setattr(self, new_api_name, new_api)
        except Exception as e:
            print(f"Exception: {e}")