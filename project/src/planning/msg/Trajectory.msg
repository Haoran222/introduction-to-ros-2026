# A short, receding-horizon trajectory produced by trajectory_planner_node: a
# time-parameterized sequence of poses/velocities the control module should track.
# header.stamp marks t=0 for every point's time_from_start.

std_msgs/Header header
TrajectoryPoint[] points
