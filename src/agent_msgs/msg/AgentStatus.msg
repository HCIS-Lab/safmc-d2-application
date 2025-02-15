uint64 timestamp                    # time since system start (microseconds)
uint8 drone_id                      # Drone ID             

geometry_msgs/Point local_position  # [m] NED local world frame

uint32 state
uint32 STATE_IDLE = 0
uint32 STATE_ARM = 1
uint32 STATE_TAKEOFF = 2
uint32 STATE_WALK_TO_SUPPLY = 3
uint32 STATE_ALIGN_TO_SUPPLY = 4
uint32 STATE_DESCEND = 5
uint32 STATE_LOAD = 6
uint32 STATE_ASCEND = 7
uint32 STATE_WALK_TO_HOTSPOT_UWB = 8
uint32 STATE_WALK_TO_HOTSPOT_ARUCO = 9
uint32 STATE_WAIT = 10
uint32 STATE_DROP = 11
uint32 STATE_BONUS = 12
uint32 STATE_HEADING_CALIBRATION = 13
