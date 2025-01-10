import rclpy
from rclpy.node import Node
from .api import DroneApi, MediatorApi
from .constants import DELTA_TIME
from .agent_machine import AgentMachine


class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        # TODO: QoS

        # context
        drone_api = DroneApi(self)
        mediator_api = MediatorApi(self)

        # machine
        self.machine = AgentMachine(drone_api, mediator_api)

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
