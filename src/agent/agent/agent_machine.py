from enum import Enum

from transitions import Machine

from api import DroneApi
from agent.behavior import (Behavior, DropBehavior, IdleBehavior, LoadBehavior,
                            TakeoffBehavior, WaitBehavior,
                            WalkToSupplyBehavior)
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
    # def __init__(self, drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):
    def __init__(self, context: Context):
        # TODO: refactor function
        def populate_triggers(transitions):
            for transition in transitions:
                transition['trigger'] = transition['dest'].name.lower()
            return transitions

        super().__init__(self, states=States,
                         transitions=populate_triggers(transitions), initial=States.IDLE)

        # self.drone_api = drone_api
        # self.mediator_api = mediator_api
        
        # self.clock = clock
        self.context = context
        self.logger = context.logger
        # init behaviors
        self.behaviors = {
            States.IDLE: IdleBehavior,
            States.TAKEOFF: TakeoffBehavior,
            States.WALK_TO_SUPPLY: WalkToSupplyBehavior,
            States.LOAD: LoadBehavior,
            States.WAIT: WaitBehavior,
            States.DROP: DropBehavior
        }

    def execute(self):
        drone_api: DroneApi = self.context.drone_api
        if True:  # TODO why?
            drone_api.maintain_offboard_control(
                self.context.get_current_timestamp())

        # 執行當前 state 任務 (一步)
        self.logger.info(f"Executing behavior for state: {self.state}")
        behavior: Behavior = self.behaviors.get(self.state)
        if behavior:
            behavior.execute(self.context)

    def proceed(self):
        # 根據條件判斷是否要 transition
        behavior: Behavior = self.behaviors.get(self.state)
        if behavior:
            trigger: str = behavior.proceed(self.context)
            if trigger:  # transition
                self.trigger(trigger)
