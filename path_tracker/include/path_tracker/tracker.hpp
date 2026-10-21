// TrackerCore: everything the ROS 2 tracker node does, minus ROS. Feed it the latest pose and it
// returns the velocity command. The benchmark and the unit tests drive the exact same class.
#pragma once
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "path_tracker/controllers.hpp"
#include "path_tracker/path.hpp"

namespace path_tracker {

struct TrackerParams {
  std::string controller = "lqr";  // "lqr" or "pure_pursuit"
  double speed = 0.4;              // m/s cruise speed
  double max_w = 2.0;              // rad/s, clamp on the command
  double max_lateral_acc = 0.6;    // m/s^2, slow down in tight turns: v <= sqrt(a / |kappa|)
  double goal_tolerance = 0.05;    // m
  double slow_down_dist = 0.5;     // m, ramp speed down before the goal
  double dt = 0.02;                // s, control period (used by LQR)
  // Pure Pursuit
  double pp_gain = 0.5, pp_min_lookahead = 0.25;
  // LQR
  double lqr_q_e = 10.0, lqr_q_th = 1.0, lqr_r = 1.0, lqr_preview = 0.15;
};

struct Command {
  double v = 0, w = 0;
  Projection proj;
  bool done = false;
};

inline std::unique_ptr<Controller> makeController(const TrackerParams& p) {
  if (p.controller == "lqr") return std::make_unique<Lqr>(p.lqr_q_e, p.lqr_q_th, p.lqr_r, p.dt, p.lqr_preview);
  if (p.controller == "pure_pursuit") return std::make_unique<PurePursuit>(p.pp_gain, p.pp_min_lookahead);
  throw std::invalid_argument("controller must be 'lqr' or 'pure_pursuit', got '" + p.controller + "'");
}

class TrackerCore {
 public:
  explicit TrackerCore(TrackerParams p) : p_(std::move(p)), ctrl_(makeController(p_)) {}

  void setPath(Path path) { path_ = std::move(path); hint_ = 0; has_path_ = true; done_ = false; }
  bool hasPath() const { return has_path_; }
  const Path& path() const { return path_; }
  const Controller& controller() const { return *ctrl_; }

  Command update(const Pose2D& pose) {
    Command c;
    if (!has_path_ || done_) { c.done = done_; return c; }
    c.proj = path_.project(pose.x, pose.y, pose.yaw, hint_);
    hint_ = c.proj.index;
    const double remaining = path_.length() - path_.s(hint_);
    const double to_end = std::hypot(path_.x(path_.size() - 1) - pose.x, path_.y(path_.size() - 1) - pose.y);
    if (remaining < p_.slow_down_dist && to_end < p_.goal_tolerance) { done_ = true; c.done = true; return c; }

    // speed: cruise, slower in tight turns (look 0.3 m ahead), ramp down near the goal
    double kmax = 0;
    for (std::size_t j = hint_; j < path_.size() && path_.s(j) - path_.s(hint_) < 0.3; ++j)
      kmax = std::max(kmax, std::abs(path_.kappa(j)));
    double v = p_.speed;
    if (kmax > 1e-6) v = std::min(v, std::sqrt(p_.max_lateral_acc / kmax));
    v = std::min(v, std::max(0.05, p_.speed * std::max(remaining, to_end) / p_.slow_down_dist));
    c.v = v;
    c.w = std::clamp(ctrl_->angularVelocity(pose, path_, c.proj, v), -p_.max_w, p_.max_w);
    return c;
  }

 private:
  TrackerParams p_;
  std::unique_ptr<Controller> ctrl_;
  Path path_;
  std::size_t hint_ = 0;
  bool has_path_ = false, done_ = false;
};

}  // namespace path_tracker
