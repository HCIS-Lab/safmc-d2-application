from enum import Enum

from transitions import Machine

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

    logger: Logger

    def __init__(self, logger: Logger, agent_parameter: AgentParameter):

        self.logger = logger

        # behavior binding
        self.state_behavior_map = {
            States.IDLE: IdleBehavior(logger),
            States.TAKEOFF: TakeoffBehavior(logger),
            States.WALK_TO_SUPPLY: WalkToSupplyBehavior(
                logger,
                navigation_goal_tolerance=agent_parameter.navigation_goal_tolerance,
            ),
            States.ALIGN_TO_SUPPLY: AlignToSupplyBehavior(
                logger,
                navigation_aruco_tolerance=agent_parameter.navigation_aruco_tolerance,
            ),
            States.LOAD: LoadBehavior(logger),
            States.WALK_TO_HOTSPOT: WalkToHotspotBehavior(logger),
            States.ALIGN_TO_HOTSPOT: AlignToHotspotBehavior(
                logger,
                navigation_aruco_tolerance=agent_parameter.navigation_aruco_tolerance,
            ),
            States.WAIT: WaitBehavior(logger),
            States.DROP: DropBehavior(logger),
            States.BONUS: BonusBehavior(logger),
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

    def execute(self):
        """
        Executes the behavior of the current state.
        """
        self.logger.info(self.state.name)
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
