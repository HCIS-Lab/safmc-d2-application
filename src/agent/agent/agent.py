import rclpy
from rclpy.node import Node

from .constants import DELTA_TIME
from .drone import Drone
from .states import States

from px4_msgs.msg import (

    OffboardControlMode,
    GotoSetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    TrajectorySetpoint

)


class Agent(Node):
    def __init__(self):
        super().__init__('agent')

        # TODO: QoS

        self.timer = self.create_timer(DELTA_TIME, self.update)
        self.drone = Drone(self)

    def handle_idle(self) -> bool:

        if (not self.drone.get_armed_status()) and \
            self.drone.get_vehicle_status().timestamp > 2000 and \
            self.drone.get_vehicle_status().pre_flight_checks_pass:
            # self.drone.get_vehicle_status().nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: why??

            self.drone.set_armed_status(True)
            self.drone.set_home_coord(self.drone.get_vehicle_local_position())

            self.drone.engage_offboard_mode()
            self.drone.arm()
            
            return True

        return False


    def update(self):

        self.drone.publish_offboard_control_heartbeat("position")

        match self.drone.state:
            case States.IDLE:
                self.get_logger().info("STATE : IDLE")
                if(self.handle_idle()):
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
