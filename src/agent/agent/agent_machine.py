from rclpy.clock import Clock
from rclpy.impl.rcutils_logger import RcutilsLogger
from enum import Enum
from transitions import Machine
from agent.behavior import (
    Behavior,
    IdleBehavior,
    TakeoffBehavior,
    WalkToSupplyBehavior,
    LoadBehavior,
    WaitBehavior,
    DropBehavior
)
from agent.api import DroneApi, MediatorApi


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
    def __init__(self, drone_api: DroneApi, mediator_api: MediatorApi, logger: RcutilsLogger, clock: Clock):

        # TODO: refactor function
        def populate_triggers(transitions):
            for transition in transitions:
                transition['trigger'] = transition['dest'].name.lower()
            return transitions

        super().__init__(self, states=States,
                         transitions=populate_triggers(transitions), initial=States.IDLE)

        self.drone_api = drone_api
        self.mediator_api = mediator_api
        self.logger = logger
        self.clock = clock

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
        if True:  # TODO why?
            self.drone_api.maintain_offboard_control(
                self.clock.now().nanoseconds)

        # 執行當前 state 任務 (一步)
        behavior: Behavior = self.behaviors.get(self.state)
        if behavior:
            behavior.execute(self.drone_api, self.mediator_api,
                             self.logger, self.clock)

    def proceed(self):
        # 根據條件判斷是否要 transition
        match self.state:
            case States.IDLE:
                if self.drone_api.is_armed:
                    self.takeoff()
            case States.TAKEOFF:
                if self.drone_api.is_altitude_reached:
                    if self.drone_api.is_loaded:
                        self.walk_to_hotspot()
                    else:
                        self.walk_to_supply()
            case States.WALK_TO_SUPPLY:
                if self.drone_api.get_supply_reached():
                    self.load()
            case States.LOAD:
                if self.drone_api.is_loaded:
                    self.walk_to_hotspot()
            case States.WALK_TO_HOTSPOT:
                pass
            case States.WAIT:
                if self.mediator_api.signal():
                    self.drop()
            case States.DROP:
                if not self.drone_api.is_loaded:
                    self.walk_to_supply()
            case _:
                pass
