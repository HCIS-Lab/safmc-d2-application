import rclpy
from rclpy.node import Node
from agent.api import DroneApi, MediatorApi
from agent.constants import DELTA_TIME
from agent.agent_machine import AgentMachine


class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        # TODO: QoS

        # logging
        logger = self.get_logger()

        # clock
        clock = self.get_clock()

        # context
        drone_api = DroneApi(self)
        mediator_api = MediatorApi(self)

        # machine
        self.machine = AgentMachine(drone_api, mediator_api, logger, clock)

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
