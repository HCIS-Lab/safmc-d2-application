from enum import Enum

from transitions import Machine

from api import ApiRegistry, Px4Api, MediatorApi

from agent.behavior import (
    AlignToHotspotBehavior,
    AlignToSupplyBehavior,
    Behavior,
    BonusBehavior,
    DropBehavior,
    IdleBehavior,
    LoadBehavior,
    TakeoffBehavior,
    WaitBehavior,
    WalkToHotspotBehavior,
    WalkToSupplyBehavior,
)
from common.logger import Logger
from agent.agent_parameter import AgentParameter


class States(Enum):
    IDLE = 0
    TAKEOFF = 2
    WALK_TO_SUPPLY = 3
    ALIGN_TO_SUPPLY = 4
    LOAD = 6
    WALK_TO_HOTSPOT = 8
    ALIGN_TO_HOTSPOT = 9
    WAIT = 10
    DROP = 11
    BONUS = 12


transitions = [
    {"source": States.IDLE, "dest": States.TAKEOFF},
    {"source": States.TAKEOFF, "dest": States.IDLE},
    {"source": States.TAKEOFF, "dest": States.WALK_TO_SUPPLY},
    {"source": States.TAKEOFF, "dest": States.ALIGN_TO_SUPPLY},
    {"source": States.TAKEOFF, "dest": States.LOAD},
    {"source": States.TAKEOFF, "dest": States.WALK_TO_HOTSPOT},
    {"source": States.TAKEOFF, "dest": States.ALIGN_TO_HOTSPOT},
    {"source": States.TAKEOFF, "dest": States.WAIT},
    {"source": States.TAKEOFF, "dest": States.DROP},
    {"source": States.TAKEOFF, "dest": States.BONUS},
    {"source": States.WALK_TO_SUPPLY, "dest": States.IDLE},
    {"source": States.WALK_TO_SUPPLY, "dest": States.ALIGN_TO_SUPPLY},
    {"source": States.ALIGN_TO_SUPPLY, "dest": States.IDLE},
    {"source": States.ALIGN_TO_SUPPLY, "dest": States.WALK_TO_SUPPLY},
    {"source": States.ALIGN_TO_SUPPLY, "dest": States.LOAD},
    {"source": States.LOAD, "dest": States.IDLE},
    {"source": States.LOAD, "dest": States.WALK_TO_HOTSPOT},
    {"source": States.WALK_TO_HOTSPOT, "dest": States.IDLE},
    {"source": States.WALK_TO_HOTSPOT, "dest": States.ALIGN_TO_HOTSPOT},
    {"source": States.ALIGN_TO_HOTSPOT, "dest": States.IDLE},
    {"source": States.ALIGN_TO_HOTSPOT, "dest": States.WALK_TO_HOTSPOT},
    {"source": States.ALIGN_TO_HOTSPOT, "dest": States.WAIT},
    {"source": States.WAIT, "dest": States.IDLE},
    {"source": States.WAIT, "dest": States.DROP},
    {"source": States.DROP, "dest": States.IDLE},
    {"source": States.DROP, "dest": States.WALK_TO_SUPPLY},
    {"source": States.BONUS, "dest": States.IDLE},
]


class AgentMachine(Machine):

    def __init__(self, agent_parameter: AgentParameter):

        # behavior binding
        self.state_behavior_map = {
            States.IDLE: IdleBehavior(),
            States.TAKEOFF: TakeoffBehavior(
                walk_goal_radius=agent_parameter.walk_goal_radius,
                takeoff_height=agent_parameter.takeoff_height,
            ),
            States.WALK_TO_SUPPLY: WalkToSupplyBehavior(
                walk_goal_radius=agent_parameter.walk_goal_radius,
                walk_speed=agent_parameter.walk_speed,
            ),
            States.ALIGN_TO_SUPPLY: AlignToSupplyBehavior(
                align_goal_radius=agent_parameter.align_goal_radius,
                align_speed=agent_parameter.align_speed,
            ),
            States.LOAD: LoadBehavior(),
            States.WALK_TO_HOTSPOT: WalkToHotspotBehavior(),
            States.ALIGN_TO_HOTSPOT: AlignToHotspotBehavior(
                align_goal_radius=agent_parameter.align_goal_radius,
                align_speed=agent_parameter.align_speed,
                align_timeout=agent_parameter.align_timeout,
            ),
            States.WAIT: WaitBehavior(),
            States.DROP: DropBehavior(),
            States.BONUS: BonusBehavior(),
        }

        # add state on_enter/on_exit callback
        states = [
            {
                "name": state_name,
                "on_enter": getattr(behavior, "on_enter", None),
                "on_exit": getattr(behavior, "on_exit", None),
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
                {**transition, "trigger": transition["dest"].name.lower()}
                for transition in transitions
            ]

        super().__init__(
            self,
            states=states,
            transitions=populate_triggers(transitions),
            initial=States.IDLE,
        )

    def pre_execute(self):
        # Logger.info(f"Current state: {self.state.name}")

        # 要 2 Hz 發送, 否則會退出 offboard control mode
        px4_api = ApiRegistry.get(Px4Api)
        px4_api.set_offboard_control_mode()

        # 傳送 agent status 給 mediator
        mediator_api = ApiRegistry.get(MediatorApi)
        mediator_api.send_status(self.state.value)

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
            if next_state := behavior.get_next_state():
                self.trigger(next_state)
