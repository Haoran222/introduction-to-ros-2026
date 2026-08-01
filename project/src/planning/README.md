# planning

Planning stack for the Autonomous Driving project. Covers the "Planning" tasks from
the project handout (`AutonomousDriving.pdf`, section 2.3):

1. **Short-term Goal Selection** — `goal_selector_node`
2. **Trajectory Planner** — `trajectory_planner_node`

A standalone geometric path planner (section 2.3.1, marked *optional* in the
handout) is not implemented: the recorded waypoint list already threads the road
network by construction, so `trajectory_planner_node` fits its spline directly
through those waypoints instead of searching a separate path over the occupancy
grid.

## Nodes

### `waypoint_recorder_node`

Not part of the driving stack — a one-off authoring tool for the "predefined poses"
waypoint list the handout describes. Subscribes to the car's true-state pose
(`/OurCar/CoM/pose`, published by the `simulation` package) and appends a waypoint
whenever the car has travelled `min_spacing` meters since the last recorded one
*or* its heading has changed by more than `corner_yaw_threshold_deg` while having
moved at least `min_corner_spacing` meters. The heading-change trigger densifies
the waypoint spacing around corners/intersections, where a spline through
widely-spaced points would otherwise cut the turn short or run wide — a fixed
distance-only spacing (e.g. one point every 3 m) is too sparse to capture a tight
turn radius while still being sensibly sparse on long straights. `output_file` is
rewritten after every new sample so a hard Ctrl+C never loses more than the
in-flight one.

Workflow to (re)generate `config/waypoints.yaml`:

```bash
ros2 launch simulation simulation.launch.py
# in another terminal:
ros2 launch planning record_waypoints.launch.py output_file:=/tmp/waypoints_recorded.yaml
# drive the car once, start marker -> goal marker (handout Fig. 1), then Ctrl+C
cp /tmp/waypoints_recorded.yaml src/planning/config/waypoints.yaml
```

Parameters: `pose_topic` (default `/OurCar/CoM/pose`), `output_file`, `min_spacing`
(default `2.0` m, straight-line spacing), `min_corner_spacing` (default `0.5` m,
densified spacing while turning), `corner_yaw_threshold_deg` (default `12.0`
degrees, heading change since the last waypoint that counts as "turning").

> **Topic name note**: the true-state pose/twist topic name is not fixed by the
> handout -- `simulation`'s `true_state_parser.h` names it after whatever Unity
> object sent the state packet. The vendored `unity_sim` build sends it as
> `OurCar/CoM` (Center of Mass), publishing `/OurCar/CoM/pose` +
> `/OurCar/CoM/twist`, *not* `/OurCar/INS/pose` as `simulation/README.md`'s
> example topic list suggests (the tf frame is still called `OurCar/INS`, only
> the topic name differs). Run `ros2 topic list | grep OurCar` after starting
> the bridge to confirm before assuming either name.

The checked-in `config/waypoints.yaml` is a **placeholder** (two dummy points), not
real track coordinates — see the comment at the top of that file.

### `goal_selector_node`

Loads the waypoint list from `waypoints_file` and, as `/OurCar/CoM/pose` updates,
tracks which waypoint is the current short-term goal. It advances when the car
gets within `arrival_radius`, and also projects the car onto a bounded window of
forward path segments to recover missed progress. The latter is important after
an avoidance manoeuvre: the car can pass a waypoint more than 1.5 m to its side,
so an arrival-radius-only selector would keep that waypoint behind the car and
eventually command a turn back toward it. The index is monotonic and the recovery
search is forward-bounded, preventing nearby roads or route crossings from
causing a backwards/far-ahead jump.

On the first pose, the selector finds the nearest route segment using vehicle
heading as a tie-breaker. This also lets the planning stack be restarted while
Unity is already partway through a drive.

Publishes:

- `/planning/next_goal` (`geometry_msgs/PoseStamped`) — the single current target,
  satisfying the handout's "you still need to select the next target from this list
  of goals" requirement.
- `/planning/local_path` (`nav_msgs/Path`) — the next `lookahead_count` waypoints
  (including the current target), for the trajectory planner to fit a
  curvature-aware path through.
- `/planning/goal_reached` (`std_msgs/Bool`) — latched true once the car reaches the
  final waypoint in the list.

Parameters: `waypoints_file` (required), `pose_topic`, `arrival_radius` (default
`1.5` m — kept well below `min_spacing` so a cluster of corner-densified waypoints
doesn't get skipped in one callback), `lookahead_count` (default `15`, to keep a
similar forward-looking *distance* now that spacing is smaller/variable),
`progress_search_count` (default `30` forward waypoints), and
`initial_heading_weight` (default `2.0` m/rad).

### `trajectory_planner_node`

Turns the local waypoint window into a kinematically/dynamically feasible,
time-parameterized trajectory:

1. Fit a uniform Catmull-Rom spline (`spline_utils.h`) from the car's current
   position through the upcoming waypoints, sampled every `sample_spacing` meters,
   with heading and signed curvature computed analytically at each sample.
2. Cap speed per-sample from a lateral-acceleration budget:
   `v <= sqrt(max_lateral_accel / |curvature|)`, clamped to `max_speed`.
3. Forward pass: cap acceleration from the car's current speed
   (`v[i] <= sqrt(v[i-1]^2 + 2*max_longitudinal_accel*ds)`).
4. Backward pass: cap deceleration so the car actually has room to slow down before
   a tight curve later in the window
   (`v[i] <= sqrt(v[i+1]^2 + 2*max_longitudinal_decel*ds)`).
5. Integrate arc length / velocity into a `time_from_start` per sample.

This curvature-based speed-profile smoothing is the same approach used by several
waypoint-following AV stacks (e.g. Autoware's waypoint planner); it avoids a
separate nonlinear MPC/optimization step while still respecting both lateral
(cornering) and longitudinal (braking/accelerating) limits, which is what the
handout asks the trajectory planner to do. Recomputed on a timer (`planning_rate_hz`,
default 10 Hz) using the latest pose/twist/local-path, so it acts as a receding
horizon planner as the car (and hence the local waypoint window) moves.
If any of those three inputs is older than `input_timeout`, the planner publishes
an empty stop trajectory instead of continuously regenerating a path from the
last Unity pose.

Publishes:

- `/planning/trajectory` (`planning/msg/Trajectory`) — for the (future) control
  module.
- `/planning/trajectory_path` (`nav_msgs/Path`) — poses only, for RViz.

Parameters: `pose_topic`, `twist_topic`, `local_path_topic`, `max_speed` (default
`4.0` m/s), `max_lateral_accel` (launch default `1.0` m/s²; node default
`1.5` m/s²), `max_longitudinal_accel`
(default `2.0` m/s²), `max_longitudinal_decel` (default `1.0` m/s², measured
for ordinary curve-speed shaping), `terminal_decel` (default `0.3` m/s²,
measured from the Unity vehicle's sustained full-brake response), and
`local_path_capacity` (default `15`, kept equal to goal selector's
`lookahead_count`).
`sample_spacing` (default `0.5` m), `planning_rate_hz` (default `10.0`).
`input_timeout` defaults to `0.5` s.

The selector uses the tight `arrival_radius` (default `1.5` m) for intermediate
waypoints so dense corner samples are not skipped, and a separate
`final_arrival_radius` (default `3.0` m) for completion after the physical
vehicle has settled under full brake.

When the local path contains fewer than `local_path_capacity` waypoints, the
planner knows the final route waypoint is visible. It constrains the final
trajectory speed to zero and propagates the conservative `terminal_decel`
backward, so the car brakes progressively instead of passing the goal at road
speed and only then receiving an empty fail-safe trajectory.

## Custom message: `planning/msg/Trajectory`

```
std_msgs/Header header
TrajectoryPoint[] points
```

```
# TrajectoryPoint.msg
geometry_msgs/Pose pose
float32 velocity
float32 curvature
float32 time_from_start
```

## Build

From the repository root:

```bash
cd project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simulation planning
source install/setup.bash
```

## Run

With the simulation bridge already running (`ros2 launch simulation simulation.launch.py`)
and a real `config/waypoints.yaml` in place (see `waypoint_recorder_node` above):

```bash
ros2 launch planning planning.launch.py
```

Inspect results:

```bash
ros2 topic echo /planning/next_goal
ros2 topic echo /planning/trajectory
ros2 run rviz2 rviz2   # add a Path display on /planning/trajectory_path
```

## Dependencies

- `ros-jazzy-yaml-cpp-vendor` (waypoint file I/O; pulls in `libyaml-cpp-dev`) — this
  ships with any standard ROS 2 install already, since ROS 2's own parameter YAML
  parsing depends on it.
