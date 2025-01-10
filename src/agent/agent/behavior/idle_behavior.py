

from .behavior import Behavior
from agent.api import DroneApi, MediatorApi


class IdleBehavior(Behavior):
    @staticmethod
    def execute(drone_api: DroneApi, mediator_api: MediatorApi):

        drone_api.get_node().get_logger().info(str(not drone_api.get_armed_status()))
        drone_api.get_node().get_logger().info(str(drone_api.get_vehicle_status().timestamp > 2000))
        drone_api.get_node().get_logger().info(str(drone_api.get_vehicle_status().pre_flight_checks_pass))


        if (not drone_api.get_armed_status()) and \
                drone_api.get_vehicle_status().timestamp > 2000 and \
                drone_api.get_vehicle_status().pre_flight_checks_pass:
            # TODO: self.drone.get_vehicle_status().nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: why??

            drone_api.set_armed_status(True)
            drone_api.set_home_coord(
                drone_api.get_vehicle_local_position())

            drone_api.engage_offboard_mode()
            drone_api.arm()
