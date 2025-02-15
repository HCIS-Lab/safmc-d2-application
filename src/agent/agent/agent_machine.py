from enum import Enum

from transitions import Machine

from agent.behavior import (
    AlignToHotspotBehavior, AlignToSupplyBehavior,
                            Behavior, DropBehavior, IdleBehavior, LoadBehavior,
                            TakeoffBehavior, WaitBehavior,
                            WalkToHotspotBehavior, WalkToSupplyBehavior, ArmBehavior)
from api import ArucoApi, DroneApi, LidarApi, MagnetApi, MediatorApi
from common.logger import Logger


class States(Enum):
    IDLE = 0
    ARM = 1
    TAKEOFF = 2
    WALK_TO_SUPPLY = 3
    ALIGN_TO_SUPPLY = 4
    LOAD = 5
    WALK_TO_HOTSPOT = 6
    ALIGN_TO_HOTSPOT = 7
    WAIT = 8
    DROP = 9


transitions = [
    {'source': States.IDLE, 'dest': States.ARM},
    {'source': States.ARM, 'dest': States.TAKEOFF},
    {'source': States.TAKEOFF, 'dest': States.WALK_TO_HOTSPOT},
    {'source': States.TAKEOFF, 'dest': States.WALK_TO_SUPPLY},
    {'source': States.WALK_TO_SUPPLY, 'dest': States.ALIGN_TO_SUPPLY},
    {'source': States.ALIGN_TO_SUPPLY, 'dest': States.WALK_TO_SUPPLY},
    {'source': States.ALIGN_TO_SUPPLY, 'dest': States.LOAD},
    {'source': States.LOAD, 'dest': States.WALK_TO_HOTSPOT},
    {'source': States.WALK_TO_HOTSPOT, 'dest': States.ALIGN_TO_HOTSPOT},
    {'source': States.ALIGN_TO_HOTSPOT, 'dest': States.WALK_TO_HOTSPOT},
    {'source': States.ALIGN_TO_HOTSPOT, 'dest': States.WAIT},
    {'source': States.WAIT, 'dest': States.DROP},
    {'source': States.DROP, 'dest': States.WALK_TO_SUPPLY},
]


class AgentMachine(Machine):
    def __init__(self, logger: Logger,
                 drone_api: DroneApi,
                 magnet_api: MagnetApi,
                 mediator_api: MediatorApi,
                 lidar_api: LidarApi,
                 aruco_api: ArucoApi):

        self.logger = logger

        # behavior binding
        self.state_behavior_map = {
            States.IDLE: IdleBehavior(logger, drone_api, mediator_api),
            States.ARM: ArmBehavior(logger, drone_api, mediator_api),
            States.TAKEOFF: TakeoffBehavior(logger, drone_api, magnet_api, mediator_api),
            States.WALK_TO_SUPPLY: WalkToSupplyBehavior(logger, drone_api, aruco_api, mediator_api),
            States.ALIGN_TO_SUPPLY: AlignToSupplyBehavior(logger, drone_api, aruco_api),
            States.LOAD: LoadBehavior(logger, drone_api, magnet_api),
            States.WALK_TO_HOTSPOT: WalkToHotspotBehavior(logger, drone_api, lidar_api, aruco_api, mediator_api),
            States.ALIGN_TO_HOTSPOT: AlignToHotspotBehavior(logger, drone_api, aruco_api),
            States.WAIT: WaitBehavior(logger, drone_api, mediator_api),
            States.DROP: DropBehavior(logger, magnet_api),
        }

        # add state on_enter/on_exit callback
        states = [
            {
                'name': state_name,
                'on_enter': getattr(behavior, 'on_enter', None),
                'on_exit': getattr(behavior, 'on_exit', None)
            }
            for state_name, behavior in self.state_behavior_map.items()
        ]

        def populate_triggers(transitions):
            """
            Adds a trigger for each transition based on the destination state.

            The trigger name is the destination state converted to lowercase.

            For example:
            - States.TAKEOFF → "takeoff"
            - States.WALK_TO_SUPPLY → "walk_to_supply"
            """
            return [
                {**transition, 'trigger': transition['dest'].name.lower()}
                for transition in transitions
            ]

        super().__init__(self, states=states,
                         transitions=populate_triggers(transitions), initial=States.IDLE)

    def execute(self):
        """
        Executes the behavior of the current state.
        """
        behavior: Behavior = self.state_behavior_map.get(self.state)
        if behavior:
            behavior.execute()

    def proceed(self):
        """
        Checks conditions and triggers state transitions.
        """
        behavior: Behavior = self.state_behavior_map.get(self.state)
        if behavior:
            if (next_state := behavior.get_next_state()):
                self.trigger(next_state)
                # TODO better log location?
                self.logger.ori.info(f"current state: {self.state}")
