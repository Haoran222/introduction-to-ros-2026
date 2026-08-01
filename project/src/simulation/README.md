# Simulation Package — ROS2 Jazzy Port

This is a ROS2 Jazzy port of the original ROS1 `simulation` package for the Unity simulator bridge.

## What was ported

- `VehicleControl.msg` ROS interface generation
- Unity TCP sensor receiver
  - True state / pose and twist
  - RGB camera
  - Depth camera
  - IMU
- UDP command transmitter
- JSON config reader
- ROS2 Python launch files
- RViz config converted to RViz2-style plugin names where possible
- Test scripts preserved

The old Catkin/libsocket setup was replaced with `ament_cmake` and standard POSIX sockets, so no external `libsocket` download is required.

## Dependencies

Install ROS2 Jazzy and then:

```bash
sudo apt install \
  ros-jazzy-rclcpp \
  ros-jazzy-std-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-rosidl-default-generators \
  ros-jazzy-ament-index-cpp \
  ros-jazzy-rviz2
```

## Build

From the repository root:

```bash
cd project
source /opt/ros/jazzy/setup.bash
colcon build --packages-select simulation
source install/setup.bash
```

## Launch

Run the bridge only:

```bash
ros2 launch simulation simulation.launch.py
```

or with an explicit Unity config file:

```bash
ros2 launch simulation simulation.launch.py \
  config_file:=src/simulation/unity_sim/Build_Ubuntu/AD_Sim_Data/StreamingAssets/simulation_config.json
```

The TCP receiver binds to `0.0.0.0` by default and reads the sensor port from `sensorConfig.port` if a config file exists. Unity must connect to the ROS2 machine's IP address and that port.

## Command topic

The UDP command transmitter subscribes to:

```text
/car_command
```

Message type:

```text
simulation/msg/VehicleControl
```

ROS2 field names are lowercase:

```text
float32 throttle
float32 steering
float32 brake
float32 reserved
```

Example:

```bash
ros2 topic pub /car_command simulation/msg/VehicleControl \
  "{throttle: 0.2, steering: 0.0, brake: 0.0, reserved: 0.0}"
```

## Sensor topics

Topics are created dynamically from Unity object names -- the true-state pose/twist
topic in particular is named after whatever object Unity's `TCPServer.cs` tags the
state packet with, **not** a fixed "INS" name. As observed with the vendored
`unity_sim` build (`ros2 topic list | grep OurCar` after starting the bridge):

```text
/OurCar/Sensors/RGBCameraLeft/image_raw
/OurCar/Sensors/RGBCameraLeft/camera_info
/OurCar/Sensors/DepthCamera/image_raw
/OurCar/Sensors/DepthCamera/camera_info
/OurCar/Sensors/IMU
/OurCar/CoM/pose
/OurCar/CoM/twist
```

The tf frame published alongside it is still hardcoded to `OurCar/INS`
(`true_state_parser.h`), so tf consumers (e.g. `perception`'s
`sensor_tf_broadcaster_node`) are unaffected -- only nodes that subscribe to the
pose/twist *topic* directly (e.g. `planning`'s nodes) need to point at
`/OurCar/CoM/pose` / `/OurCar/CoM/twist` instead of `/OurCar/INS/...`. Always
verify with `ros2 topic list` rather than assuming either name.

## Notes

- The original ROS1 launch files are kept as `*.ros1` references.
- Unity is not started automatically in the ROS2 launch file. Start Unity separately unless you adapt `scripts/run_unity.sh` to your local Unity build path.
- The JSON parameter reader is retained as a diagnostic/config reader. In ROS2, parameters are node-local, so the receiver and transmitter also read the config file themselves.
