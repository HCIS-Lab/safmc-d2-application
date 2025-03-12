import rclpy
from rclpy.node import Node

from agent.agent_machine import AgentMachine
from agent.constants import DELTA_TIME
from api import ApiRegistry, ArucoApi, LidarApi, MagnetApi, MediatorApi, Px4Api
from common.logger import Logger


class Agent(Node):

    logger: Logger

    def __init__(self):
        super().__init__("agent")
        self.logger = Logger(self.get_logger(), self.get_clock())

        self._register_apis()
        self.machine = AgentMachine(self.logger)
        self.timer = self.create_timer(DELTA_TIME, self._update)

    def _register_apis(self):
        ApiRegistry.register(Px4Api, self)
        ApiRegistry.register(MediatorApi, self)
        ApiRegistry.register(MagnetApi, self)
        ApiRegistry.register(LidarApi, self)
        ApiRegistry.register(ArucoApi, self)

    def _update(self):
        # TODO: 進入 offboard 再搞
        # 要 2 Hz 發送, 否則會退出 offboard control mode
        px4_api = ApiRegistry.get(Px4Api)
        px4_api.set_offboard_control_mode()

        # 傳送 agent status 給 mediator
        mediator_api = ApiRegistry.get(MediatorApi)
        mediator_api.send_status(self.machine.state)
        mediator_api.send_agent_local_position(px4_api.local_position)

        self.machine.proceed()
        self.machine.execute()


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
