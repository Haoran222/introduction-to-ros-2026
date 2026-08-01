# perception

Perception stack for the Autonomous Driving project. Covers the three "Perception"
tasks from the project handout (`AutonomousDriving.pdf`, section 2.2.3):

1. **Coordinate Frame Build-up** — `sensor_tf_broadcaster_node`
2. **Occupancy Map Generation** — `depth_to_pointcloud_node` + `octomap_server` (external package)
3. **Traffic Light Recognition** — `traffic_light_detector_node`

## Nodes

### `sensor_tf_broadcaster_node`

Publishes the static tf2 tree from `OurCar/INS` down to every sensor frame, using
`tf2_ros::StaticTransformBroadcaster`. The dynamic `world -> OurCar/INS` transform is
already published by the `simulation` package's `Unity_ROS_message_Rx` node
(`true_state_parser.h`), so this node only fills in the rigid, never-changing part of
the tree:

```
world                                    (dynamic, from simulation)
└── OurCar/INS
    ├── OurCar/Sensors/IMU               (co-located with the INS)
    └── OurCar/Center                    (vehicle footprint projection)
        └── OurCar/Sensors/SensorBase
            ├── OurCar/Sensors/SemanticCamera
            ├── OurCar/Sensors/DepthCamera
            ├── OurCar/Sensors/RGBCameraLeft
            └── OurCar/Sensors/RGBCameraRight
```

All offsets default to the values measured off the handout's vehicle dimension
drawings (Fig. 2/3): INS is 0.718 m above the footprint, the sensor cluster sits
0.82 m ahead of the INS and 1.269 m above the footprint, and the left/right RGB
cameras are offset 0.2 m to either side of the centered depth/semantic camera. Every
offset is a ROS parameter, so the geometry can be re-tuned without recompiling.

The camera frames use the standard REP-103 *optical frame* convention (Z forward,
X right, Y down), which is what `depth_to_pointcloud_node` and any future
`image_geometry`/`depth_image_proc`-based code expects.

### `depth_to_pointcloud_node`

Subscribes to the depth camera's `image_raw` (`16UC1`, millimeters) and
`camera_info`, and re-projects each valid pixel into a 3D point using the pinhole
model (`fx`, `fy`, `cx`, `cy` from `camera_info.k`). Publishes a
`sensor_msgs/PointCloud2` on `points`, in the depth camera's own optical frame.

Parameters:

- `depth_topic` (default `/OurCar/Sensors/DepthCamera/image_raw`)
- `pixel_stride` (default `4`) — subsamples the depth image for real-time performance
- `min_range` / `max_range` (meters) — drops points outside the useful sensor range

A pixel value of `0` means "no return" (matches how `simulation`'s
`depth_camera_parser.h` encodes max-range/invalid pixels) and is skipped.

### Occupancy map: `octomap_server`

We rely on the standard `octomap_server` package (`ros-jazzy-octomap-server`) to turn
the point cloud stream into a voxel-grid occupancy map, as suggested by the handout.
It is wired up in `launch/perception.launch.py` with `cloud_in` remapped to
`/perception/points` and `frame_id: world`; it uses tf2 (via the frames above) to
transform each cloud into the world frame before inserting it into the octree.

Because the simulation config's `groundRemoval: true` already strips the road out of
the depth image on the Unity side (see the project handout, section 3), no separate
ground-plane filtering is done here.

### `traffic_light_detector_node`

Detects traffic light state from a plain RGB camera image using classical computer
vision (HSV color thresholding + blob shape filtering), so it works even with the
semantic camera disabled (handout bonus: "solving the problem without using semantic
camera").

Steps per frame: crop to the upper 45% of the image → threshold red/yellow/green
in HSV → keep circular, filled blobs surrounded by the simulator's distinctive
yellow/orange traffic-light housing → require at least two separate, vertically
aligned housing components around every color candidate → prefer an illuminated
red/green lens over the larger fixed yellow housing highlights → debounce over a
few frames before changing the reported state.

The housing check replaces an earlier dark-surround heuristic. Live testing at
the first intersection showed that the old heuristic rejected the real lenses
inside their bright yellow housing, then classified an NPC's circular red
tail-light as a red signal and stopped forever. In the captured failure frame,
the real red lens had a 0.37 yellow-housing surround fraction while both NPC
tail-light candidates measured 0.00.

Publishes:

- `traffic_light/state` (`perception/msg/TrafficLightState`) — full detection result
- `traffic_light/must_stop` (`std_msgs/Bool`) — convenience signal for the decision
  making / state machine module (true on RED; optionally also YELLOW)
- `traffic_light/debug_image` (`sensor_msgs/Image`, if `publish_debug_image:=true`) —
  annotated view for tuning, viewable with `rqt_image_view`

Parameters: `image_topic`, `roi_top_fraction`, `roi_bottom_fraction`,
`roi_left_fraction`, `roi_right_fraction`, `min_blob_area`,
`min_red_blob_area`, `min_circularity`, `min_color_fill_fraction`,
`min_yellow_housing_fraction`, `max_yellow_housing_fraction`,
`debounce_frames`, `stop_on_yellow` (default `false`), `publish_debug_image`.
The Unity model's inactive reflectors are yellow/orange, so yellow remains
visible on the diagnostic state but does not request a stop unless
`stop_on_yellow:=true`. The housing fraction has both a
minimum and maximum: a real lens has a yellow ring, while a tiny blob embedded
in an almost-solid yellow wall/pipe is rejected. The component-stack check also
rejects circular restaurant/road signs whose yellow decoration previously
looked like a signal lens.

## Custom message: `perception/msg/TrafficLightState`

```
uint8 UNKNOWN=0
uint8 RED=1
uint8 YELLOW=2
uint8 GREEN=3

std_msgs/Header header
uint8 state
bool detected
float32 confidence
int32 pixel_area
```

This is the project's "implement your own message type" deliverable.

## Build

From the repository root:

```bash
cd project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simulation perception
source install/setup.bash
```

## Run

With the simulation bridge already running (`ros2 launch simulation simulation.launch.py`):

```bash
ros2 launch perception perception.launch.py
```

Inspect results:

```bash
ros2 topic echo /traffic_light/state
ros2 run rqt_image_view rqt_image_view   # view /traffic_light/debug_image
ros2 run tf2_tools view_frames           # dump the tf tree to frames.pdf
```

## Dependencies

- `ros-jazzy-cv-bridge`, `ros-jazzy-image-transport`, OpenCV (already present with
  a desktop ROS install)
- `ros-jazzy-octomap-server`, `ros-jazzy-octomap`, `ros-jazzy-octomap-msgs`

```bash
sudo apt install ros-jazzy-octomap-server ros-jazzy-octomap ros-jazzy-octomap-msgs
```
