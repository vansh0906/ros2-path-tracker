// Reference path: arc length, heading and curvature computed from a list of points.
// No ROS dependency, so the same code runs in the ROS 2 nodes, the benchmark and the unit tests.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace path_tracker {

constexpr double kPi = 3.14159265358979323846;

inline double wrapAngle(double a) {
  while (a > kPi) a -= 2 * kPi;
  while (a < -kPi) a += 2 * kPi;
  return a;
}

struct Projection {
  std::size_t index = 0;   // closest path sample
  double cross_track = 0;  // signed distance, positive when the robot is LEFT of the path
  double heading_error = 0;
};

class Path {
 public:
  Path() = default;

  // Builds the path from points; consecutive duplicates are dropped.
  Path(const std::vector<double>& xs, const std::vector<double>& ys) {
    if (xs.size() != ys.size()) throw std::invalid_argument("x and y sizes differ");
    for (std::size_t i = 0; i < xs.size(); ++i) {
      if (!x_.empty() && std::hypot(xs[i] - x_.back(), ys[i] - y_.back()) < 1e-6) continue;
      x_.push_back(xs[i]);
      y_.push_back(ys[i]);
    }
    if (x_.size() < 3) throw std::invalid_argument("a path needs at least 3 distinct points");
    const std::size_t n = x_.size();
    s_.assign(n, 0.0);
    yaw_.assign(n, 0.0);
    kappa_.assign(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) s_[i] = s_[i - 1] + std::hypot(x_[i] - x_[i - 1], y_[i] - y_[i - 1]);
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t a = i == 0 ? 0 : i - 1, b = i + 1 == n ? n - 1 : i + 1;
      yaw_[i] = std::atan2(y_[b] - y_[a], x_[b] - x_[a]);
    }
    // Menger curvature (circle through three neighbouring points), signed: positive = turning left
    for (std::size_t i = 1; i + 1 < n; ++i) {
      const double ax = x_[i] - x_[i - 1], ay = y_[i] - y_[i - 1];
      const double bx = x_[i + 1] - x_[i], by = y_[i + 1] - y_[i];
      const double cx = x_[i + 1] - x_[i - 1], cy = y_[i + 1] - y_[i - 1];
      const double cross = ax * by - ay * bx;
      const double denom = std::hypot(ax, ay) * std::hypot(bx, by) * std::hypot(cx, cy);
      kappa_[i] = denom > 1e-12 ? 2.0 * cross / denom : 0.0;
    }
    kappa_[0] = kappa_[1];
    kappa_[n - 1] = kappa_[n - 2];
    // light moving-average smoothing: planner paths are often jagged
    std::vector<double> sm(n);
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t lo = i < 3 ? 0 : i - 3, hi = std::min(n - 1, i + 3);
      double sum = 0;
      for (std::size_t j = lo; j <= hi; ++j) sum += kappa_[j];
      sm[i] = sum / static_cast<double>(hi - lo + 1);
    }
    kappa_ = sm;
  }

  std::size_t size() const { return x_.size(); }
  double length() const { return s_.back(); }
  double x(std::size_t i) const { return x_[i]; }
  double y(std::size_t i) const { return y_[i]; }
  double yaw(std::size_t i) const { return yaw_[i]; }
  double kappa(std::size_t i) const { return kappa_[i]; }
  double s(std::size_t i) const { return s_[i]; }

  // Index of the first sample at least `ds` metres of arc length after sample i.
  std::size_t ahead(std::size_t i, double ds) const {
    const double target = s_[i] + ds;
    auto it = std::lower_bound(s_.begin() + static_cast<std::ptrdiff_t>(i), s_.end(), target);
    return it == s_.end() ? size() - 1 : static_cast<std::size_t>(it - s_.begin());
  }

  // Closest sample, searched in a window around `hint` so a figure-8 crossing never jumps lanes.
  Projection project(double px, double py, double pyaw, std::size_t hint, double back = 0.5,
                     double fwd = 2.0) const {
    const double s_lo = s_[hint] - back, s_hi = s_[hint] + fwd;
    std::size_t best = hint;
    double best_d = 1e18;
    for (std::size_t i = hint; i-- > 0 && s_[i] >= s_lo;) consider(i, px, py, best, best_d);
    for (std::size_t i = hint; i < size() && s_[i] <= s_hi; ++i) consider(i, px, py, best, best_d);
    // refine onto the neighbouring segment so errors change smoothly (no jumps between samples)
    Projection p;
    p.index = best;
    double fx = x_[best], fy = y_[best], fyaw = yaw_[best];
    for (int side : {-1, 1}) {
      const std::ptrdiff_t j = static_cast<std::ptrdiff_t>(best) + side;
      if (j < 0 || j >= static_cast<std::ptrdiff_t>(size())) continue;
      const std::size_t a = side < 0 ? static_cast<std::size_t>(j) : best, b = side < 0 ? best : static_cast<std::size_t>(j);
      const double sx = x_[b] - x_[a], sy = y_[b] - y_[a];
      const double t = ((px - x_[a]) * sx + (py - y_[a]) * sy) / (sx * sx + sy * sy);
      if (t < 0.0 || t > 1.0) continue;
      fx = x_[a] + t * sx;
      fy = y_[a] + t * sy;
      fyaw = yaw_[a] + t * wrapAngle(yaw_[b] - yaw_[a]);
      break;
    }
    const double dx = px - fx, dy = py - fy;
    p.cross_track = -std::sin(fyaw) * dx + std::cos(fyaw) * dy;
    p.heading_error = wrapAngle(pyaw - fyaw);
    return p;
  }

 private:
  void consider(std::size_t i, double px, double py, std::size_t& best, double& best_d) const {
    const double d = std::hypot(px - x_[i], py - y_[i]);
    if (d < best_d) { best_d = d; best = i; }
  }
  std::vector<double> x_, y_, s_, yaw_, kappa_;
};

// Test paths that fit in a small lab: a figure-8 (lemniscate of Bernoulli) and a rounded rectangle.
// Both start at (0, 0) heading along +x, which is where a real robot's odometry starts.
inline Path makeFigureEight(double half_width = 1.6, int laps = 1, double spacing = 0.02) {
  std::vector<double> rx, ry;
  const int n = static_cast<int>(std::ceil(5.3 * half_width / spacing));
  for (int k = 0; k <= n; ++k) {             // one loop, starting where the two lobes cross
    const double t = kPi / 2 + 2 * kPi * k / n;
    const double d = 1 + std::sin(t) * std::sin(t);
    rx.push_back(half_width * std::cos(t) / d);
    ry.push_back(half_width * std::sin(t) * std::cos(t) / d);
  }
  // rotate so the path leaves the origin heading along +x
  const double h = std::atan2(ry[1] - ry[0], rx[1] - rx[0]);
  const double c = std::cos(-h), sn = std::sin(-h);
  std::vector<double> xs, ys;
  for (int lap = 0; lap < laps; ++lap)
    for (std::size_t i = (lap == 0 ? 0 : 1); i < rx.size(); ++i) {
      xs.push_back(c * rx[i] - sn * ry[i]);
      ys.push_back(sn * rx[i] + c * ry[i]);
    }
  return Path(xs, ys);
}

inline Path makeRoundedLoop(double length = 3.0, double width = 1.5, double radius = 0.5, int laps = 1,
                            double spacing = 0.02) {
  // counter-clockwise rounded rectangle starting at (0, 0) heading +x
  std::vector<double> xs, ys;
  const double sx = length - 2 * radius, sy = width - 2 * radius;
  auto line = [&](double x0, double y0, double x1, double y1) {
    const int n = std::max(1, static_cast<int>(std::hypot(x1 - x0, y1 - y0) / spacing));
    for (int k = 0; k < n; ++k) { xs.push_back(x0 + (x1 - x0) * k / n); ys.push_back(y0 + (y1 - y0) * k / n); }
  };
  auto arc = [&](double cx, double cy, double a0) {
    const int n = std::max(4, static_cast<int>(kPi / 2 * radius / spacing));
    for (int k = 0; k < n; ++k) {
      const double a = a0 + kPi / 2 * k / n;
      xs.push_back(cx + radius * std::cos(a));
      ys.push_back(cy + radius * std::sin(a));
    }
  };
  for (int lap = 0; lap < laps; ++lap) {
    line(0, 0, sx / 2, 0);
    arc(sx / 2, radius, -kPi / 2);
    line(sx / 2 + radius, radius, sx / 2 + radius, radius + sy);
    arc(sx / 2, radius + sy, 0);
    line(sx / 2, width, -sx / 2, width);
    arc(-sx / 2, radius + sy, kPi / 2);
    line(-sx / 2 - radius, radius + sy, -sx / 2 - radius, radius);
    arc(-sx / 2, radius, kPi);
    line(-sx / 2, 0, 0, 0);
  }
  xs.push_back(0);
  ys.push_back(0);
  return Path(xs, ys);
}

}  // namespace path_tracker
