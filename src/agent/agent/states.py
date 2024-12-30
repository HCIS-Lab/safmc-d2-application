from enum import Enum


class States(Enum):
    IDLE = 0
    TAKEOFF = 1
    WALK_TO_SUPPLY = 2
    LOAD = 3
    WALK_TO_HOTSPOT = 4
    WAIT = 5
    DROP = 6


transitions = [
    {
        'source': States.IDLE,
        'dest': States.TAKEOFF,
        'trigger': States.TAKEOFF.name.lower(),
    },
    {
        'source': States.TAKEOFF,
        'dest': States.WALK_TO_SUPPLY,
        'trigger': States.WALK_TO_SUPPLY.name.lower(),
    },
    {
        'source': States.TAKEOFF,
        'dest': States.WALK_TO_HOTSPOT,
        'trigger': States.WALK_TO_HOTSPOT.name.lower(),
    },
    {
        'source': States.WALK_TO_SUPPLY,
        'dest': States.LOAD,
        'trigger': States.LOAD.name.lower(),
    },
    {
        'source': States.LOAD,
        'dest': States.WALK_TO_HOTSPOT,
        'trigger': States.WALK_TO_HOTSPOT.name.lower(),
    },
    {
        'source': States.WALK_TO_HOTSPOT,
        'dest': States.WAIT,
        'trigger': States.WAIT.name.lower(),
    },
    {
        'source': States.WAIT,
        'dest': States.DROP,
        'trigger': States.DROP.name.lower(),
    },
    {
        'source': States.DROP,
        'dest': States.WALK_TO_SUPPLY,
        'trigger': States.WALK_TO_SUPPLY.name.lower(),
    },
]
