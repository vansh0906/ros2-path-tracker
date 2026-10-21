// Path-tracking controllers for a differential-drive (unicycle) robot that takes /cmd_vel.
// Both return an angular velocity for a given forward speed.
#pragma once
#include <array>
#include <cmath>
#include <map>
#include <string>

#include "path_tracker/path.hpp"

namespace path_tracker {

struct Pose2D { double x = 0, y = 0, yaw = 0; };

class Controller {
 public:
  virtual ~Controller() = default;
  virtual std::string name() const = 0;
  // returns angular velocity (rad/s) for forward speed v (m/s)
  virtual double angularVelocity(const Pose2D& pose, const Path& path, const Projection& proj, double v) = 0;
};

// Pure Pursuit: drive an arc through a point `lookahead` metres ahead on the path.
class PurePursuit : public Controller {
 public:
  PurePursuit(double gain = 0.5, double min_lookahead = 0.25) : gain_(gain), min_ld_(min_lookahead) {}
  std::string name() const override { return "Pure Pursuit"; }
  double lookahead(double v) const { return gain_ * std::abs(v) + min_ld_; }

  double angularVelocity(const Pose2D& pose, const Path& path, const Projection& proj, double v) override {
    const double ld = lookahead(v);
    const std::size_t j = path.ahead(proj.index, ld);
    const double dx = path.x(j) - pose.x, dy = path.y(j) - pose.y;
    const double alpha = wrapAngle(std::atan2(dy, dx) - pose.yaw);
    const double dist = std::max(std::hypot(dx, dy), 1e-3);
    return v * 2.0 * std::sin(alpha) / dist;   // arc curvature times speed
  }

 private:
  double gain_, min_ld_;
};

// LQR on the linearised path-error model plus curvature feed-forward.
//   state x = [cross-track error e, heading error th],  input u = w - v * kappa
//   e[k+1] = e + v dt th + 0.5 v dt^2 u,   th[k+1] = th + dt u
class Lqr : public Controller {
 public:
  Lqr(double q_e = 10.0, double q_th = 1.0, double r = 1.0, double dt = 0.02, double preview_s = 0.15)
      : q_e_(q_e), q_th_(q_th), r_(r), dt_(dt), preview_(preview_s) {}
  std::string name() const override { return "LQR + feed-forward"; }

  // Gain K (u = -K x) from the discrete algebraic Riccati equation, solved by fixed-point iteration.
  std::array<double, 2> gain(double v) const {
    const double key = std::round(std::abs(v) * 100.0) / 100.0;
    auto it = cache_.find(key);
    if (it != cache_.end()) return it->second;
    const double vv = std::max(key, 0.05);
    const double a01 = vv * dt_, b0 = 0.5 * vv * dt_ * dt_, b1 = dt_;
    double p00 = q_e_, p01 = 0, p11 = q_th_;
    std::array<double, 2> K{0, 0};
    for (int it_n = 0; it_n < 20000; ++it_n) {
      // A^T P A, A^T P B, B^T P B with A = [[1, a01], [0, 1]], B = [b0, b1]^T
      const double pb0 = p00 * b0 + p01 * b1, pb1 = p01 * b0 + p11 * b1;   // P B
      const double bpb = b0 * pb0 + b1 * pb1;
      const double apb0 = pb0, apb1 = a01 * pb0 + pb1;                     // A^T P B
      const double s = r_ + bpb;
      const double apa00 = p00, apa01 = a01 * p00 + p01, apa11 = a01 * a01 * p00 + 2 * a01 * p01 + p11;
      const double n00 = q_e_ + apa00 - apb0 * apb0 / s;
      const double n01 = apa01 - apb0 * apb1 / s;
      const double n11 = q_th_ + apa11 - apb1 * apb1 / s;
      const double change = std::abs(n00 - p00) + std::abs(n01 - p01) + std::abs(n11 - p11);
      p00 = n00; p01 = n01; p11 = n11;
      K = {apb0 / s, apb1 / s};
      if (change < 1e-10 * (1 + std::abs(p00))) break;
    }
    cache_[key] = K;
    return K;
  }

  double angularVelocity(const Pose2D&, const Path& path, const Projection& proj, double v) override {
    const auto K = gain(v);
    // feed-forward from curvature slightly ahead, to cover the robot's response lag
    const double kappa = path.kappa(path.ahead(proj.index, std::abs(v) * preview_));
    return v * kappa - (K[0] * proj.cross_track + K[1] * proj.heading_error);
  }

 private:
  double q_e_, q_th_, r_, dt_, preview_;
  mutable std::map<double, std::array<double, 2>> cache_;
};

}  // namespace path_tracker
