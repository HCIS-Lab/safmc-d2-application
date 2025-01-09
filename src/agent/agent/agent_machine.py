from enum import Enum
from transitions import Machine
from api import DroneApi, MediatorApi
from behavior import WaitBehavior, DropBehavior


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
    def __init__(self, drone_api: DroneApi, mediator_api: MediatorApi):

        # TODO: refactor function
        def populate_triggers(transitions):
            for transition in transitions:
                transition['trigger'] = transition['dest'].name.lower()
            return transitions

        super().__init__(self, states=States,
                         transitions=populate_triggers(transitions), initial=States.IDLE)

        self.drone_api = drone_api
        self.mediator_api = mediator_api

        # init behaviors
        self.behaviors = {
            States.WAIT: WaitBehavior,
            States.DROP: DropBehavior
        }

    def execute(self):
        # 執行當前 state 任務 (一步)
        behavior = self.behaviors.get(self.state)
        if behavior:
            behavior.execute(self.drone_api, self.mediator_api)

    def proceed(self):
        # 根據條件判斷是否要 transition
        match self.state:
            case States.IDLE:
                if False:
                    self.takeoff()
            case States.TAKEOFF:
                pass
            case States.WALK_TO_SUPPLY:
                pass
            case States.LOAD:
                pass
            case States.WALK_TO_HOTSPOT:
                pass
            case States.WAIT:
                if self.mediator_api.get_signal():
                    self.drop()
            case States.DROP:
                if self.drone_api.is_payload_dropped():
                    self.walk_to_supply()
            case _:
                pass
