from enum import Enum

from transitions import Machine
from functools import partial

from agent.behavior import (Behavior, DropBehavior, IdleBehavior, LoadBehavior,
                            TakeoffBehavior, WaitBehavior,
                            WalkToSupplyBehavior)
from api import DroneApi
from common.context import Context


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
    def __init__(self, context: Context):
        # TODO: refactor function
        def populate_triggers(transitions):
            for transition in transitions:
                transition['trigger'] = transition['dest'].name.lower()
            return transitions

        # self.clock = clock
        self.context = context
        self.logger = context.logger
        # init behaviors
        self.state_behavior_map = {
            States.IDLE: IdleBehavior(),
            States.TAKEOFF: TakeoffBehavior(),
            States.WALK_TO_SUPPLY: WalkToSupplyBehavior(),
            States.LOAD: LoadBehavior(),
            States.WAIT: WaitBehavior(),
            States.DROP: DropBehavior()
        }

        # TODO

        states = []
        for state_name, behavior in self.state_behavior_map.items():
            # Initialize behavior instance
            state = {'name': state_name}
            print(behavior)
            if hasattr(behavior, 'on_enter'):
                state['on_enter'] = partial(behavior.on_enter, context)
            if hasattr(behavior, 'on_exit'):
                state['on_exit'] = partial(behavior.on_exit, context)
            states.append(state)
        
        print(states)
        super().__init__(self, states=states,
                         transitions=populate_triggers(transitions), initial=States.IDLE)
        
        # exit(0)

    def execute(self):
        drone_api: DroneApi = self.context.drone_api

        drone_api.maintain_offboard_control(
            self.context.get_current_timestamp())

        # 執行當前 state 任務 (一步)
        self.context.log_info(f"Current state: {self.state.name}")
        behavior: Behavior = self.state_behavior_map.get(self.state)
        if behavior:
            behavior.execute(self.context)

    def proceed(self):
        # 根據條件判斷是否要 transition
        behavior: Behavior = self.state_behavior_map.get(self.state)
        if behavior:
            if (next_state := behavior.get_next_state(self.context)):
                self.trigger(next_state)
