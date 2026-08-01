# decision_making

Decision-making stack for the Autonomous Driving project. Covers the "Decision
Making" tasks from the project handout (`AutonomousDriving.pdf`, section 2.4):

1. **Traffic Lights** (required) — stop at red/yellow, regardless of direction.
2. **Optional Event I: Vehicle Merging** — slow for an NPC merging into the lane.
3. **Optional Event II: Emergency Brake** — hard stop for a car braking ahead.
4. **State Machine** (optional in the handout, implemented here) — `decision_state_machine_node`.

## Design

Both optional events are handled by the *same* generic "is something in my lane,
how far, closing how fast" signal from `forward_obstacle_monitor_node` — there is
no event-specific scripting (no hardcoded timing/location for "the merge event" vs
"the emergency-brake event"). A moderate, distant hazard produces `CAUTION`
(matches Event I: slow down, don't necessarily stop); a close or fast-closing
hazard produces `EMERGENCY_STOP` (matches Event II). The same logic would also
correctly react to e.g. a stalled car anywhere else on the track.

`decision_state_machine_node` sits between `planning` and the (future) control
module: it republishes `/planning/trajectory` with velocities scaled down or
zeroed depending on state, rather than modifying `trajectory_planner_node` itself.
This keeps the perception -> planning -> decision-making -> control layering the
handout describes, and matches this project's existing per-package separation
(see `perception/README.md`, `planning/README.md`).

## Nodes

### `forward_obstacle_monitor_node`

Finds the nearest obstacle ahead of the car, within a lane-width corridor, using
the depth camera's **current-frame** point cloud: perception's `/perception/points`
(depth-camera optical frame), transformed into `world` via `tf2_ros::Buffer` +
`TransformListener` (latest available transform, not exact-stamp, via
`tf2::TimePointZero`).

**Design history**: the first version used `octomap_server`'s accumulated
`/octomap_point_cloud_centers` instead, reasoning that it was already in `world`
(no tf needed) and reused perception's existing occupancy-map pipeline. **This
was wrong for hazard detection** and was found live: a merging/braking NPC
produced zero `CAUTION`/`EMERGENCY_STOP` before a collision. An accumulated
occupancy map needs several hits at the same voxel before trusting it as
occupied — correct for static-scene mapping, too laggy for a fast-moving
obstacle, which may already be gone (or have already hit the car) by the time
enough hits accumulate. Switched to the raw per-frame cloud so hazard detection
reacts to what the sensor sees *right now*; occupancy-map-building remains a
separate, valid use of the same sensor for the planning/path side (see
`perception/README.md`), just the wrong tool for real-time collision avoidance.

Per point cloud message: project every point onto the current planned trajectory
(falling back to the car's forward/right axes if no fresh trajectory exists);
keep points ahead (`min_forward_distance` to `max_range`), within
`lane_half_width`, and between `min_obstacle_height` and
`max_obstacle_height`; publish the nearest distance plus a closing speed derived
from consecutive distances.

This is purely geometric — no semantic camera, no NPC-specific classification —
consistent with `perception`'s traffic light detector's approach to the same bonus
requirement (solving perception without the semantic camera).

Publishes `/decision_making/hazard_status` (`decision_making/msg/HazardStatus`).

Parameters: `points_topic` (default `/perception/points`), `world_frame` (default
`world`), `pose_topic` (default `/OurCar/CoM/pose`), `lane_half_width` (default
`1.75` m — car is `2.0` m wide per the handout's Fig. 2, so this gives a bit of
margin), `max_range` (default `40.0` m), `min_forward_distance` (default `0.5` m,
excludes points essentially on top of the car/sensor mount),
`min_obstacle_height` (default `0.5` m), and `max_obstacle_height` (default
`1.8` m). The upper filter removes overhead beams; a live false stop was traced
to a crossbeam at world z=`5.87` m. At lateral offsets beyond
`edge_lateral_start` (default `1.3` m), `edge_min_obstacle_height` (default
`0.8` m) replaces the lower threshold; this rejects low roadside dividers at
the corridor boundary while retaining passenger-car bodies.

### `decision_state_machine_node`

Six states, highest-severity wins each tick (10 Hz timer over the latest cached
signals):

| State            | Trigger                                                                 | Trajectory velocity  |
|-------------------|-------------------------------------------------------------------------|-----------------------|
| `SENSOR_FAULT`    | hazard or planning trajectory has timed out                              | x 0 |
| `EMERGENCY_STOP`  | hazard within `emergency_distance`, or closing with time-to-collision < `emergency_ttc` | x 0 |
| `TRAFFIC_STOP`    | perception's `/traffic_light/must_stop` is true                        | x 0                   |
| `AVOIDING`        | persistent nearly-stationary hazard with a clear adjacent corridor       | x `avoid_speed_factor`, path shifted sideways |
| `CAUTION`         | hazard within `caution_distance`                                        | x `clamp(distance / caution_distance, caution_min_speed_factor, 1.0)` |
| `DRIVE`           | none of the above                                                        | x 1 (unchanged)       |

`EMERGENCY_STOP` outranks `TRAFFIC_STOP`: a collision is worse than an
illegal-but-controlled stop past a light. Verified by manual injection (see
"Testing" below): both states individually zero the trajectory correctly, and
triggering both signals at once resolves to `EMERGENCY_STOP` as intended.

**Hysteresis (added after a live test drive caused a real loss of control)**: a
candidate state must hold steady for `state_hold_time` (default `0.3` s) before
it's actually adopted. Without this, perception's `must_stop` flickering faster
than its own 3-frame debounce could catch (3 consecutive camera frames can be
under 200ms) chattered the gated trajectory's speed between full-speed and
full-stop every control tick — the car crashed into a wall shortly after.
`EMERGENCY_STOP` is exempt from this delay *on entry* (a real imminent-collision
reading must never be delayed), but is then **sticky** for at least
`emergency_min_dwell` (default `1.0` s) before hysteresis-gated exit applies —
without this second layer, a hazard distance sitting right at
`emergency_distance` chattered EMERGENCY_STOP/CAUTION every ~0.5s from sensor
noise alone (observed live as the car lurching full-throttle/full-brake
repeatedly).

Once `AVOIDING` is committed, the selected side is fixed for that manoeuvre.
A brief `avoid_clear=false` reading is tolerated for `avoid_clear_loss_hold`
(default `0.7` s), preventing point-cloud noise from switching immediately
between `AVOIDING` and `EMERGENCY_STOP`. A sustained blocked corridor or a
fast-closing obstacle still enters `EMERGENCY_STOP`. Entry remains limited by
`avoid_max_closing_speed` (`0.3` m/s). Once committed, a raw nearest-point
closing-speed spike must exceed `avoid_abort_closing_speed` (`1.5` m/s) for
`avoid_abort_closing_hold` (`0.5` s) before aborting the manoeuvre; this rejects
single-frame depth-cloud identity jumps without masking a sustained approach.

Hazard and trajectory receipt times are checked independently. If either input
is older than its timeout, the state becomes `SENSOR_FAULT` immediately and an
empty/zero-speed trajectory is published. The vehicle therefore never resumes
from a stale "last clear" frame.

Publishes:

- `/decision_making/trajectory` (`planning/msg/Trajectory`) — consumed by
  `control`'s `trajectory_follower_node` instead of `/planning/trajectory`
  directly.
- `/decision_making/state` (`std_msgs/String`) — `"DRIVE"` / `"TRAFFIC_STOP"` /
  `"CAUTION"` / `"EMERGENCY_STOP"` / `"AVOIDING"` / `"SENSOR_FAULT"`, for
  debugging/RViz/the presentation demo.

Parameters: `must_stop_topic` (default `/traffic_light/must_stop`), `hazard_topic`
(default `/decision_making/hazard_status`), `trajectory_topic` (default
`/planning/trajectory`), `caution_distance` (default `15.0` m),
`caution_min_speed_factor` (default `0.3`), `emergency_distance` (default `6.0`
m), `emergency_ttc` (default `2.0` s), `decision_rate_hz` (default `10.0`),
`state_hold_time` (default `0.3` s), `emergency_min_dwell` (default `1.0` s).
Avoidance stability and freshness parameters include `avoid_clear_loss_hold`
(default `0.7` s), `avoid_abort_closing_speed` (default `1.5` m/s),
`avoid_abort_closing_hold` (default `0.5` s), `hazard_timeout` (default `1.0`
s), and `trajectory_timeout` (default `1.0` s).

**Simplification**: velocities are scaled in place without recomputing each
point's `time_from_start` against the new (lower) speed, so those timestamps go
stale once gating is active for more than an instant. Acceptable for now since a
controller is expected to track `(pose, velocity)` per point rather than replay
against wall-clock time; revisit if the control module ends up needing accurate
timing under sustained gating.

## Custom message: `decision_making/msg/HazardStatus`

```
std_msgs/Header header
float32 distance         # meters to nearest obstacle ahead; large sentinel if none
float32 closing_speed    # m/s, positive = obstacle getting closer
bool detected
```

## Known limitations

- The lane corridor is a straight strip along the car's *current* heading. On a
  sharp curve, a genuinely-ahead-on-the-road obstacle can fall outside the
  corridor (false negative), and something geometrically ahead-in-a-straight-line
  but not actually on the drivable path could fall inside it (false positive).
  Good enough for the handout's straight-lane merge/brake events; would need a
  path-relative (not heading-relative) corridor for sharper curves.
- The per-frame point cloud has no temporal smoothing beyond the state machine's
  own hysteresis (see above) -- a single occluded/empty frame reads as
  "no hazard" immediately. Considered acceptable: a real approaching obstacle
  stays in view over many consecutive frames, so a one-frame gap doesn't change
  the outcome, and erring toward responsiveness suits collision avoidance better
  than erring toward a stale reading (the opposite tradeoff of the accumulated
  occupancy map this replaced -- see `forward_obstacle_monitor_node` above).

## Build

From the repository root:

```bash
cd project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simulation perception planning decision_making control
source install/setup.bash
```

## Run

With `simulation`, `perception`, and `planning` already running:

```bash
ros2 launch decision_making decision_making.launch.py
```

`control`'s `trajectory_follower_node` should be launched after this to actually
drive the car (see `control/README.md`) -- `decision_making` alone only computes
and publishes the gated trajectory, it doesn't send anything to the simulator.

Inspect results:

```bash
ros2 topic echo /decision_making/hazard_status
ros2 topic echo /decision_making/state
ros2 topic echo /decision_making/trajectory
```

## Testing

No live NPC events were available while this was built (`eventEnable` was
temporarily `[false, false]` for waypoint recording — see `planning/README.md`).
The state machine was instead verified by publishing synthetic `HazardStatus` /
`must_stop` messages directly (with the real `forward_obstacle_monitor_node` /
`traffic_light_detector_node` briefly stopped so the injected values weren't
immediately overwritten by their real, hazard-free feed) and checking
`/decision_making/state` and the resulting scaled `/decision_making/trajectory`
velocities:

- `distance: 10.0` (between `emergency_distance` and `caution_distance`) -> `CAUTION`, velocities scaled by exactly `10/15 = 0.667`.
- `distance: 3.0` -> `EMERGENCY_STOP`, velocities zeroed.
- `must_stop: true` -> `TRAFFIC_STOP`, velocities zeroed.
- `distance: 3.0` and `must_stop: true` together -> `EMERGENCY_STOP` (correct priority).

### Live testing with real `eventEnable: [true, true]` NPC events

Once `control` existed, the full stack was driven live against the real
simulator with real traffic lights and NPC events enabled. This is what actually
found both bugs described above (accumulated-map lag missing an NPC; `must_stop`
chatter causing a wall collision) -- the synthetic injection tests above did not
and could not have caught either one, since both are about the *timing/staleness*
of real sensor data, not the state machine's logic given a clean signal. After
both fixes, a manual injection re-test of the sticky-`EMERGENCY_STOP` behavior
confirmed it holds steady (no chatter) for 30+ seconds against a real, persistent
close-range hazard reading.

**Open issue found in the same live session, not yet root-caused**: the car
steers hard left immediately from a standing start instead of going roughly
straight. The `VehicleControl.steering` sign convention was verified correct by
directly publishing a known positive value to `/car_command` and confirming the
car's yaw decreased (= turned right, matching `dummy_controller`'s "positive =
right" comment and `control`'s negation logic) -- so this is not a sign-inversion
bug in either package. Suspected but unconfirmed: the car's actual Unity spawn
position was observed drifting several meters between restarts from
`config/waypoints.yaml`'s recorded start point, which could make an initial hard
turn the *correct* pure-pursuit response (rejoining an offset path) rather than a
bug -- needs checking against a fresh, precisely-measured spawn pose before
`control` starts driving, which wasn't completed before this session ended
(Unity became unresponsive after repeated restarts -- see the WSL2 notes in the
top-level `src/README.md`'s "Unity TCP Notes" section).
