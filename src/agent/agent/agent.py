import rclpy
from rclpy.node import Node

from agent.agent_machine import AgentMachine
from agent.agent_parameter import AgentParameter
from api import ApiRegistry, ArucoApi, LidarApi, MagnetApi, MediatorApi, Px4Api, UwbApi
from common.logger import Logger


class Agent(Node):
    def __init__(self):
        super().__init__("agent")
        Logger(self)

        self._register_apis()
        agent_parameter = AgentParameter(self)
        self.machine = AgentMachine(agent_parameter)
        self.timer = self.create_timer(agent_parameter.delta_time, self._update)

    def _register_apis(self):
        ApiRegistry.register(Px4Api, self)
        ApiRegistry.register(MediatorApi, self)
        ApiRegistry.register(MagnetApi, self)
        ApiRegistry.register(LidarApi, self)
        ApiRegistry.register(ArucoApi, self)
        ApiRegistry.register(UwbApi, self)

    def _update(self):
        self.machine.proceed()
        self.machine.pre_execute()
        self.machine.execute()
        # self.machine.post_execute()


def main(args=None):

    rclpy.init(args=args)
    agent = Agent()

    try:
        rclpy.spin(agent)
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
