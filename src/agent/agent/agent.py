import rclpy
from rclpy.node import Node

from agent.agent_machine import AgentMachine
from agent.constants import DELTA_TIME
from api import DroneApi, MagnetApi, MediatorApi
from common.context import Context
from common.parameters import get_parameter


class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        # TODO: QoS?

        drone_id = get_parameter(self, 'drone_id', 0)
        group_id = get_parameter(self, 'group_id', 0)

        # TODO: check param (drone_id, group_id) is unique

        self.api_context = Context(
            self.get_logger(),
            self.get_clock(),
            drone_api=DroneApi(self),
            mediator_api=MediatorApi(self, drone_id, group_id),
            magnet_api=MagnetApi(self)
        )

        # machine
        self.machine = AgentMachine(self.api_context)

        self.timer = self.create_timer(DELTA_TIME, self.update)

    def update(self):
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


if __name__ == '__main__':
    main()
