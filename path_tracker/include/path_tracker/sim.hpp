// Closed-loop simulation helpers shared by the benchmark and the tests (no ROS needed).
#pragma once
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "path_tracker/tracker.hpp"
#include "path_tracker/unicycle.hpp"

namespace path_tracker {

// Gaussian noise from raw mt19937 output (Box-Muller), so results match on GCC, Clang and MSVC.
// std::normal_distribution is implemented differently by each standard library.
class Noise {
 public:
  explicit Noise(std::uint32_t seed) : gen_(seed) {}
  double gauss(double sigma) {
    const double u1 = (static_cast<double>(gen_()) + 1.0) / 4294967297.0;
    const double u2 = static_cast<double>(gen_()) / 4294967296.0;
    return sigma * std::sqrt(-2.0 * std::log(u1)) * std::cos(2 * kPi * u2);
  }
 private:
  std::mt19937 gen_;
};

struct SimConfig {
  double start_offset = 0.2;  // m, robot starts this far left of the path
  double odom_xy_noise = 0.0; // m, std dev added to each pose measurement
  double odom_yaw_noise = 0.0;// rad
  double max_time = 120.0;    // s
  int substeps = 5;           // physics steps per control step
  std::uint32_t seed = 1;
  RobotLimits limits{};
};

struct SimResult {
  bool finished = false;
  double time = 0, settle = 0, rms_cte = 0, max_cte = 0, mean_w_rate = 0;
  std::vector<double> t, x, y, cte, w;
};

inline SimResult simulate(const Path& path, const TrackerParams& tp, const SimConfig& cfg) {
  TrackerCore tracker(tp);
  tracker.setPath(path);
  const double y0 = path.yaw(0);
  Pose2D start{path.x(0) - std::sin(y0) * cfg.start_offset, path.y(0) + std::cos(y0) * cfg.start_offset, y0};
  Unicycle robot(start, cfg.limits);
  Noise noise(cfg.seed);
  SimResult r;
  double prev_w = 0, w_rate_sum = 0;
  int steps = 0;
  for (double t = 0; t < cfg.max_time; t += tp.dt) {
    Pose2D meas = robot.pose();
    meas.x += noise.gauss(cfg.odom_xy_noise);
    meas.y += noise.gauss(cfg.odom_xy_noise);
    meas.yaw = wrapAngle(meas.yaw + noise.gauss(cfg.odom_yaw_noise));
    const Command cmd = tracker.update(meas);
    if (cmd.done) { r.finished = true; r.time = t; break; }
    for (int k = 0; k < cfg.substeps; ++k) robot.step(cmd.v, cmd.w, tp.dt / cfg.substeps);
    // score against the TRUE pose
    const Projection truth = tracker.path().project(robot.pose().x, robot.pose().y, robot.pose().yaw, cmd.proj.index);
    r.t.push_back(t); r.x.push_back(robot.pose().x); r.y.push_back(robot.pose().y);
    r.cte.push_back(truth.cross_track); r.w.push_back(cmd.w);
    w_rate_sum += std::abs(cmd.w - prev_w) / tp.dt; prev_w = cmd.w; ++steps;
  }
  if (!r.finished) r.time = cfg.max_time;
  // settle: the robot starts 0.2 m off the path; time until it is first within 3 cm of it
  r.settle = r.time;
  for (std::size_t i = 0; i < r.cte.size(); ++i)
    if (std::abs(r.cte[i]) < 0.03) { r.settle = r.t[i]; break; }
  // tracking errors are scored after the first 3 s, once the start offset has been removed
  const std::size_t from = static_cast<std::size_t>(3.0 / tp.dt);
  double sq = 0; std::size_t n = 0;
  for (std::size_t i = from; i < r.cte.size(); ++i) {
    sq += r.cte[i] * r.cte[i];
    r.max_cte = std::max(r.max_cte, std::abs(r.cte[i]));
    ++n;
  }
  r.rms_cte = n ? std::sqrt(sq / static_cast<double>(n)) : 0;
  r.mean_w_rate = steps ? w_rate_sum / steps : 0;
  return r;
}

}  // namespace path_tracker
