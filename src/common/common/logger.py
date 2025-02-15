import inspect

from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger


class Logger():
    def __init__(self, rcutils_logger: RcutilsLogger, clock: Clock):
        self.ori: RcutilsLogger = rcutils_logger
        self.__clock: Clock = clock
        self.__log_caller_time_map = {}
        self.__time_interval = 5000000000  # TODO magic number

    def debug(self, msg, **kwargs):
        self.__log(self.ori.debug, msg, **kwargs)

    def info(self, msg, **kwargs):
        self.__log(self.ori.info, msg, **kwargs)

    def warning(self, msg, **kwargs):
        self.__log(self.ori.warning, msg, **kwargs)

    def error(self, msg, **kwargs):
        self.__log(self.ori.error, msg, **kwargs)

    def fatal(self, msg, **kwargs):
        self.__log(self.ori.fatal, msg, **kwargs)

    def __log(self, log_function: callable, msg, **kwargs):
        frame = inspect.currentframe().f_back
        caller = (frame.f_code.co_filename, frame.f_lineno)
        timestamp = self.__clock.now().nanoseconds
        last_time = self.__log_caller_time_map.get(caller)
        if last_time is None or (timestamp - last_time >= self.__time_interval):
            self.__log_caller_time_map[caller] = timestamp
            # log_function(msg, **kwargs)
            log_function(msg)
