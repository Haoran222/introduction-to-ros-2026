# A single sample of a planned trajectory: where the car should be, how fast it
# should be going, the local path curvature there, and when it should arrive
# (relative to the trajectory's header.stamp).

geometry_msgs/Pose pose
float32 velocity
float32 curvature
float32 time_from_start
