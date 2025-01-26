from enum import Enum

from transitions import Machine

from agent.behavior import (Behavior, DropBehavior, IdleBehavior, LoadBehavior,
                            TakeoffBehavior, WaitBehavior, WalkToHotspotBehavior,
                            WalkToSupplyBehavior)
from api import DroneApi, MagnetApi, MediatorApi
from common.logger import Logger


class States(Enum):
    IDLE = 0
    TAKEOFF = 1
    WALK_TO_SUPPLY = 2
    LOAD = 3
    WALK_TO_HOTSPOT = 4
    WAIT = 5
    DROP = 6


transitions = [
    {'source': States.IDLE, 'dest': States.TAKEOFF},
    {'source': States.TAKEOFF, 'dest': States.WALK_TO_SUPPLY},
    {'source': States.TAKEOFF, 'dest': States.WALK_TO_HOTSPOT},
    {'source': States.WALK_TO_SUPPLY, 'dest': States.LOAD},
    {'source': States.LOAD, 'dest': States.WALK_TO_HOTSPOT},
    {'source': States.WALK_TO_HOTSPOT, 'dest': States.WAIT},
    {'source': States.WAIT, 'dest': States.DROP},
    {'source': States.DROP, 'dest': States.WALK_TO_SUPPLY},
]


class AgentMachine(Machine):
    def __init__(self, logger: Logger, drone_api: DroneApi, magnet_api: MagnetApi, mediator_api: MediatorApi):
        
        self.logger = logger

        def populate_triggers(transitions): # TODO: refactor function
            for transition in transitions:
                transition['trigger'] = transition['dest'].name.lower()
            return transitions

        # init behaviors
        self.state_behavior_map = {
            States.IDLE: IdleBehavior(logger, drone_api),
            States.TAKEOFF: TakeoffBehavior(logger, drone_api, magnet_api),
            States.WALK_TO_SUPPLY: WalkToSupplyBehavior(logger, drone_api),
            States.LOAD: LoadBehavior(logger, drone_api, magnet_api),
            States.WALK_TO_HOTSPOT: WalkToHotspotBehavior(logger, drone_api),
            States.WAIT: WaitBehavior(logger, drone_api, mediator_api),
            States.DROP: DropBehavior(logger, magnet_api)
        }

        # TODO

        states = []
        for state_name, behavior in self.state_behavior_map.items():
            # Initialize behavior instance
            state = {'name': state_name}
            if hasattr(behavior, 'on_enter'):
                state['on_enter'] = behavior.on_enter
            if hasattr(behavior, 'on_exit'):
                state['on_exit'] = behavior.on_exit
            states.append(state)

        super().__init__(self, states=states,
                         transitions=populate_triggers(transitions), initial=States.IDLE)

    def execute(self):
        # 執行當前 state 任務 (一步)
        behavior: Behavior = self.state_behavior_map.get(self.state)
        if behavior:
            behavior.execute()

    def proceed(self):
        # 根據條件判斷是否要 transition
        behavior: Behavior = self.state_behavior_map.get(self.state)
        if behavior:
            if (next_state := behavior.get_next_state()):
                self.trigger(next_state)
                self.logger.ori.info(f"current state: {self.state}")
