

from .behavior import Behavior
from api import DroneApi, MediatorApi


class IdleBehavior(Behavior):
    @staticmethod
    def execute(self, drone_api: DroneApi, mediator_api: MediatorApi):

        if (not drone_api.get_armed_status()) and \
                drone_api.get_vehicle_status().timestamp > 2000 and \
                drone_api.get_vehicle_status().pre_flight_checks_pass:
            # TODO: self.drone.get_vehicle_status().nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: why??

            drone_api.set_armed_status(True)
            drone_api.set_home_coord(
                drone_api.get_vehicle_local_position())

            drone_api.engage_offboard_mode()
            drone_api.arm()
