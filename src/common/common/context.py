from typing import Optional, Type

from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger

from api.api import Api

import inspect

class Context():
    def __init__(self, logger: RcutilsLogger, clock: Clock, **apis):
        self.logger: RcutilsLogger = logger
        self.clock: Clock = clock
        for api_name, api in apis.items():
            setattr(self, api_name, api)
        
        self.log_caller_time_map = {}

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

    def log_info(self, msg, **kwargs):
        frame = inspect.currentframe().f_back
        caller = (frame.f_code.co_filename, frame.f_lineno)
        
        current_time = self.get_current_timestamp()
        
        last_time = self.log_caller_time_map.get(caller)
        if last_time is None or (current_time - last_time >= 5000000000): # TODO magic number
            self.log_caller_time_map[caller] = current_time
            self.logger.info(msg, **kwargs)
