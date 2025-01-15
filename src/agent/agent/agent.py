import rclpy
from rclpy.node import Node
from agent.api import DroneApi, MediatorApi
from agent.constants import DELTA_TIME
from agent.agent_machine import AgentMachine
from agent.common.context import Context


class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        # TODO: QoS?

        self.context = Context(
            self.get_logger(), 
            self.get_clock(), 
            drone_api=DroneApi(self),
            mediator_api=MediatorApi(self)
        )

        # machine
        self.machine = AgentMachine(self.context)

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
