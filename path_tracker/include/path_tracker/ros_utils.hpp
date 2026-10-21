// Small helpers shared by the ROS 2 nodes.
#pragma once
#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"

namespace path_tracker {

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace path_tracker
