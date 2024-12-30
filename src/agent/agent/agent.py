import rclpy
from rclpy.node import Node

from .constants import DELTA_TIME
from .drone import Drone
from .states import States


class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        # TODO: QoS

        self.timer = self.create_timer(DELTA_TIME, self.update)
        self.drone = Drone(self)

    def update(self):
        match self.drone.state:
            case States.IDLE:
                self.drone.takeoff()

            case States.TAKEOFF:
                if True:
                    self.drone.walk_to_supply()
                else:
                    self.drone.walk_to_hotspot()

            case States.WALK_TO_SUPPLY:
                self.drone.load()

            case States.LOAD:
                self.drone.walk_to_hotspot()

            case States.WALK_TO_HOTSPOT:
                self.drone.wait()

            case States.WAIT:
                self.drone.drop()

            case States.DROP:
                self.drone.walk_to_supply()

            case _:
                pass


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
