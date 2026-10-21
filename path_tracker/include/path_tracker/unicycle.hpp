// Simple differential-drive robot model: velocity commands go through a first-order lag,
// acceleration limits and an angular-velocity limit, like a real motor controller.
#pragma once
#include <algorithm>
#include <cmath>

#include "path_tracker/controllers.hpp"

namespace path_tracker {

struct RobotLimits {
  double max_v = 1.0;        // m/s
  double max_w = 2.0;        // rad/s   (TurtleBot 4 / ROSMASTER X3 class)
  double max_acc = 1.0;      // m/s^2
  double max_alpha = 4.0;    // rad/s^2
  double time_constant = 0.15;  // s, velocity response lag
};

class Unicycle {
 public:
  explicit Unicycle(Pose2D start = {}, RobotLimits lim = {}) : pose_(start), lim_(lim) {}

  void step(double cmd_v, double cmd_w, double dt) {
    cmd_v = std::clamp(cmd_v, -lim_.max_v, lim_.max_v);
    cmd_w = std::clamp(cmd_w, -lim_.max_w, lim_.max_w);
    const double k = std::min(1.0, dt / lim_.time_constant);
    const double dv = std::clamp(k * (cmd_v - v_), -lim_.max_acc * dt, lim_.max_acc * dt);
    const double dw = std::clamp(k * (cmd_w - w_), -lim_.max_alpha * dt, lim_.max_alpha * dt);
    v_ += dv;
    w_ += dw;
    // exact integration of a constant-twist arc over dt
    if (std::abs(w_) > 1e-9) {
      const double r = v_ / w_;
      pose_.x += r * (std::sin(pose_.yaw + w_ * dt) - std::sin(pose_.yaw));
      pose_.y -= r * (std::cos(pose_.yaw + w_ * dt) - std::cos(pose_.yaw));
    } else {
      pose_.x += v_ * dt * std::cos(pose_.yaw);
      pose_.y += v_ * dt * std::sin(pose_.yaw);
    }
    pose_.yaw = wrapAngle(pose_.yaw + w_ * dt);
  }

  const Pose2D& pose() const { return pose_; }
  double v() const { return v_; }
  double w() const { return w_; }

 private:
  Pose2D pose_;
  RobotLimits lim_;
  double v_ = 0, w_ = 0;
};

}  // namespace path_tracker
