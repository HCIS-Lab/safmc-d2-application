import rclpy
from rclpy.node import Node

from agent.agent_machine import AgentMachine
from agent.constants import DELTA_TIME
from api import DroneApi, LidarApi, MagnetApi, MediatorApi
from common.logger import Logger
from common.parameters import get_parameter


class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        self.logger = Logger(self.get_logger(), self.get_clock())

        drone_id = get_parameter(self, 'drone_id', 1)
        group_id = get_parameter(self, 'group_id', 1)
        
        self.logger.info(f"New agent: drone_id={drone_id}, group_id={group_id}")
        
        self.drone_api = DroneApi(self, drone_id)
        self.mediator_api = MediatorApi(self, drone_id, group_id)
        self.magnet_api = MagnetApi(self)
        self.lidar_api = LidarApi(self, drone_id)
        # machine
        self.machine = AgentMachine(
            self.logger, self.drone_api, self.magnet_api, self.mediator_api, self.lidar_api)

        self.timer = self.create_timer(DELTA_TIME, self.update)

    def update(self):
        # 要 2 Hz 發送, 否則會退出 offboard control mode
        self.drone_api.set_offboard_control_mode()
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
