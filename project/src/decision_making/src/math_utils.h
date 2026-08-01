#pragma once

// Convert a quaternion to planar yaw.

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>

namespace decision_making {

inline double QuaternionToYaw(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace decision_making
