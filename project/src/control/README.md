# control

Vehicle control for the Autonomous Driving project. Covers the handout's
"Vehicle Control" section (`AutonomousDriving.pdf`, section 2.5): a control
algorithm that follows the planned trajectory by outputting throttle, brake, and
steering.

This is the last stage of the pipeline: `simulation` -> `perception` ->
`planning` -> `decision_making` -> **`control`** -> back into `simulation` via
`/car_command`.

## Node: `trajectory_follower_node`

- **Lateral control**: adaptive Pure Pursuit. The lookahead target point is
  chosen by walking the arc length of `/decision_making/trajectory` until it
  exceeds a speed-scaled lookahead distance (`lookahead_min` at low
  speed, growing toward `lookahead_max` at high speed via `lookahead_speed_gain`
  -- keeps steering from being over-aggressive when it matters least and
  under-reactive when it matters most). Steering angle follows directly from
  bicycle-model geometry using the known wheelbase (`2.63` m -- handout Fig. 3:
  rear-axle-to-INS `1.35` m + INS-to-front-axle `1.28` m).
- **Longitudinal control**: PID on `(target speed - current speed)`. Positive
  desired acceleration maps to throttle, negative to brake (never both at once).

**Important design point, found by testing against the live simulator**: the
speed target is the minimum planned velocity from
`speed_preview_min_distance` through the steering lookahead plus
`speed_preview_extra_distance`, **not** simply the steering target or
`trajectory.points[0]`. This preserves a tight curve's low-speed sample even
when the steering point has already reached the faster straight after it, while
still skipping points[0]'s current-speed boundary condition so the car can
accelerate from rest.

**Fail-safe**: trajectory, pose, and twist must all have arrived within
`command_timeout`; otherwise the node publishes a hard stop
(`throttle=0, steering=0, brake=1`) instead of acting on stale simulation data.
A zero-speed decision is also held with full brake and neutral steering rather
than being treated as a zero PID error. This is also the state at startup,
before the first complete input set arrives.

Publishes `VehicleControl` on the relative topic `car_command` (matches
`dummy_controller`'s and `simulation`'s convention -- resolves to `/car_command`
with no additional namespacing).

### Parameters

| Parameter | Default | Notes |
|---|---|---|
| `trajectory_topic` | `/decision_making/trajectory` | the *gated* trajectory, not `/planning/trajectory` directly |
| `pose_topic` | `/OurCar/CoM/pose` | see the topic-naming note in `simulation/README.md` |
| `twist_topic` | `/OurCar/CoM/twist` | |
| `wheelbase` | `2.63` m | known from the handout's vehicle dimensions |
| `max_steering_angle` | `0.6` rad | **assumed** -- the simulator's mapping from the normalized `steering` command to a physical front-wheel angle isn't documented; tune against real driving behavior |
| `lookahead_min` / `lookahead_max` / `lookahead_speed_gain` | `2.0` / `8.0` m / `0.6` | tighter arc-length lookahead to avoid cutting city corners |
| `speed_preview_min_distance` / `speed_preview_extra_distance` | `1.0` / `5.0` m | ignore the current-speed boundary point and preview farther for curve braking |
| `speed_kp` / `speed_ki` / `speed_integral_limit` | `0.6` / `0.1` / `2.0` | longitudinal PID |
| `accel_to_throttle_gain` / `accel_to_brake_gain` | `1.0` / `1.0` | **assumed** -- the simulator's accel-per-normalized-command isn't documented either; tune similarly |
| `control_rate_hz` | `20.0` | |
| `command_timeout` | `0.5` s | |
| `stop_velocity_epsilon` | `0.05` m/s | targets at or below this value are held with full brake |

## Build

From the repository root:

```bash
cd project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simulation perception planning decision_making control
source install/setup.bash
```

## Run

With `simulation`, `perception`, `planning`, and `decision_making` already
running:

```bash
ros2 launch control control.launch.py
```

`dummy_controller` must **not** be running at the same time -- both publish to
`/car_command` and would fight each other.

Inspect results:

```bash
ros2 topic echo /car_command
ros2 topic hz /car_command
```

## Testing

Smoke-tested live against the running Unity simulator (not synthetic data): from
a standing start at the spawn/start marker, the car accelerated, tracked the
recorded waypoint path around the first stretch of track (confirmed by comparing
`/OurCar/CoM/pose` against the known waypoint shape), and slowed appropriately
through the first curve. The points[0]-vs-lookahead speed-target bug above was
caught exactly this way -- the car moved a few centimeters and stopped, which a
synthetic single-shot test would not have revealed.

Terminal-stop profiles are distinguished from decision stops: a decision stop
has every trajectory velocity at zero and applies full brake immediately; a
terminal profile has only its final point at zero, so the speed preview follows
the progressively decreasing planned speeds until the endpoint enters the
normal steering lookahead. This prevents stopping several metres before the
goal merely because the longer curve preview can already see the final zero.

Not yet tested: a full lap end-to-end including all four traffic-light
intersections and both optional NPC events, or tuning `max_steering_angle` /
`accel_to_*_gain` against real driving feel (defaults are untuned first guesses).
