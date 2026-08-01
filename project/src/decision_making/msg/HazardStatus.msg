# Nearest forward obstacle in the ego lane corridor, as seen in perception's
# accumulated occupancy map. Published by forward_obstacle_monitor_node, consumed
# by decision_state_machine_node to drive the CAUTION / EMERGENCY_STOP states
# (handout's Optional Events I/II: vehicle merging, emergency brake).

std_msgs/Header header
float32 distance         # meters to the nearest occupied voxel ahead; large sentinel if none detected
float32 closing_speed    # m/s, positive = obstacle getting closer (finite-difference of distance over time)
bool detected

# Signed lateral position of the nearest obstacle relative to the planned path
# (+ = right of path, matching VehicleControl's steering convention), 0 if not
# detected. Lets decision_state_machine_node work out which way to shift the
# path to get around a stopped obstacle (see avoid_clear below).
float32 lateral_offset

# True if shifting the path `avoid_shift` meters away from the obstacle's side
# (see forward_obstacle_monitor_node's `avoid_shift` param) stays clear of any
# other detected point over that same stretch of path. False (the safe
# default) if no obstacle is detected, or if the far side is blocked too.
bool avoid_clear
