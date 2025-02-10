from enum import Enum

from transitions import Machine

from agent.behavior import (Behavior, DropBehavior, IdleBehavior, LoadBehavior,
                            TakeoffBehavior, WaitBehavior,
<<<<<<< HEAD
                            WalkToHotspotBehavior, WalkToSupplyBehavior, 
                            AlignToHotspotBehavior, AlignToSupplyBehavior)
from api import DroneApi, MagnetApi, MediatorApi
=======
                            WalkToHotspotBehavior, WalkToSupplyBehavior)
from api import DroneApi, LidarApi, MagnetApi, MediatorApi
>>>>>>> origin/phase1
from common.logger import Logger


class States(Enum):
    IDLE = 0
    TAKEOFF = 1
    WALK_TO_SUPPLY = 2
    LOAD = 3
    WALK_TO_HOTSPOT = 4
    WAIT = 5
    DROP = 6
    ALIGN_TO_SUPPLY = 7
    ALIGN_TO_HOTSPOT = 8


transitions = [
    {'source': States.IDLE, 'dest': States.TAKEOFF},
    {'source': States.TAKEOFF, 'dest': States.WALK_TO_SUPPLY},
    {'source': States.TAKEOFF, 'dest': States.WALK_TO_HOTSPOT},
    {'source': States.WALK_TO_SUPPLY, 'dest': States.ALIGN_TO_SUPPLY},
    {'source': States.ALIGN_TO_SUPPLY, 'dest': States.LOAD},
    {'source': States.LOAD, 'dest': States.WALK_TO_HOTSPOT},
    {'source': States.WALK_TO_HOTSPOT, 'dest': States.ALIGN_TO_HOTSPOT},
    {'source': States.ALIGN_TO_HOTSPOT, 'dest': States.WAIT},
    {'source': States.WAIT, 'dest': States.DROP},
    {'source': States.DROP, 'dest': States.WALK_TO_SUPPLY},
]


class AgentMachine(Machine):
    def __init__(self, logger: Logger, drone_api: DroneApi, magnet_api: MagnetApi, mediator_api: MediatorApi,lidar_api: LidarApi):

        self.logger = logger

        # behavior binding
        self.state_behavior_map = {
            States.IDLE: IdleBehavior(logger, drone_api),
            States.TAKEOFF: TakeoffBehavior(logger, drone_api, magnet_api),
            States.WALK_TO_SUPPLY: WalkToSupplyBehavior(logger, drone_api),
            States.LOAD: LoadBehavior(logger, drone_api, magnet_api),
            States.WALK_TO_HOTSPOT: WalkToHotspotBehavior(logger, drone_api,lidar_api),
            States.WAIT: WaitBehavior(logger, drone_api, mediator_api),
            States.DROP: DropBehavior(logger, magnet_api),
            States.ALIGN_TO_SUPPLY: AlignToSupplyBehavior(logger, drone_api),
            States.ALIGN_TO_HOTSPOT: AlignToHotspotBehavior((logger, drone_api)) 
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
