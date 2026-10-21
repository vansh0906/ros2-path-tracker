// Unit tests for the ROS-independent core (run by colcon test / ctest).
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "path_tracker/sim.hpp"

using namespace path_tracker;

static Path circle(double r, int n = 2000) {
  std::vector<double> xs, ys;
  for (int k = 0; k <= n; ++k) {
    const double a = -kPi / 2 + 2 * kPi * k / n;
    xs.push_back(r * std::cos(a));
    ys.push_back(r + r * std::sin(a));
  }
  return Path(xs, ys);
}

static Path straightLine(double length = 6.0) {
  std::vector<double> xs, ys;
  for (int k = 0; k <= 600; ++k) { xs.push_back(length * k / 600.0); ys.push_back(0.0); }
  return Path(xs, ys);
}

TEST(Path, CircleCurvatureIsOneOverRadius) {
  const Path p = circle(1.25);
  EXPECT_NEAR(p.length(), 2 * kPi * 1.25, 1e-3);
  for (std::size_t i = 10; i + 10 < p.size(); i += 97) EXPECT_NEAR(p.kappa(i), 0.8, 0.01);
}

TEST(Path, CrossTrackIsPositiveOnTheLeft) {
  const Path p = straightLine();
  EXPECT_NEAR(p.project(2.0, 0.3, 0.0, 0).cross_track, 0.3, 1e-9);
  EXPECT_NEAR(p.project(2.0, -0.2, 0.0, 0).cross_track, -0.2, 1e-9);
  EXPECT_NEAR(p.project(2.0, 0.0, 0.4, 0).heading_error, 0.4, 1e-9);
}

TEST(Path, ProjectionIsSmoothBetweenSamples) {
  const Path p = circle(1.0, 300);  // coarse samples on purpose
  double prev = p.project(0.0, 0.1, 0.0, 0).cross_track;
  std::size_t hint = 0;
  for (int k = 1; k < 200; ++k) {
    const double a = -kPi / 2 + 0.5 * kPi * k / 200;
    const Projection pr = p.project(0.9 * std::cos(a), 1.0 + 0.9 * std::sin(a), a + kPi / 2, hint);
    hint = pr.index;
    EXPECT_NEAR(pr.cross_track, 0.1, 1e-3);           // always 0.1 m inside the circle
    EXPECT_LT(std::abs(pr.cross_track - prev), 1e-3);  // no jumps from one sample to the next
    prev = pr.cross_track;
  }
}

TEST(Path, TestPathsStartAtOriginHeadingForward) {
  for (const Path& p : {makeFigureEight(), makeRoundedLoop()}) {
    EXPECT_NEAR(p.x(0), 0.0, 1e-9);
    EXPECT_NEAR(p.y(0), 0.0, 1e-9);
    EXPECT_NEAR(p.yaw(0), 0.0, 0.02);
  }
}

TEST(Lqr, ClosedLoopIsStableAtEverySpeed) {
  Lqr lqr;
  const double dt = 0.02;
  for (double v : {0.1, 0.3, 0.6, 1.0}) {
    const auto K = lqr.gain(v);
    EXPECT_GT(K[0], 0.0);
    EXPECT_GT(K[1], 0.0);
    // eigenvalues of A - B K must lie inside the unit circle
    const double a00 = 1 - 0.5 * v * dt * dt * K[0], a01 = v * dt - 0.5 * v * dt * dt * K[1];
    const double a10 = -dt * K[0], a11 = 1 - dt * K[1];
    const double tr = a00 + a11, det = a00 * a11 - a01 * a10;
    const double disc = tr * tr - 4 * det;
    const double mag = disc >= 0 ? std::max(std::abs((tr + std::sqrt(disc)) / 2), std::abs((tr - std::sqrt(disc)) / 2))
                                 : std::sqrt(det);
    EXPECT_LT(mag, 1.0) << "unstable at v = " << v;
  }
}

TEST(Unicycle, StraightAndFullCircle) {
  RobotLimits lim; lim.time_constant = 1e-6; lim.max_acc = lim.max_alpha = 1e9;
  Unicycle a({0, 0, 0}, lim);
  for (int k = 0; k < 100; ++k) a.step(0.5, 0.0, 0.01);
  EXPECT_NEAR(a.pose().x, 0.5, 1e-6);
  EXPECT_NEAR(a.pose().y, 0.0, 1e-9);
  Unicycle b({0, 0, 0}, lim);
  const double w = 1.0, T = 2 * kPi / w;
  const int n = 10000;
  for (int k = 0; k < n; ++k) b.step(0.5, w, T / n);
  EXPECT_NEAR(b.pose().x, 0.0, 1e-3);
  EXPECT_NEAR(b.pose().y, 0.0, 1e-3);
}

TEST(Tracker, RejectsUnknownController) {
  TrackerParams p; p.controller = "mpc";
  EXPECT_THROW(TrackerCore{p}, std::invalid_argument);
}

TEST(Tracker, BothControllersConvergeOnAStraightLine) {
  for (const char* c : {"pure_pursuit", "lqr"}) {
    TrackerParams tp; tp.controller = c; tp.speed = 0.5;
    const SimResult r = simulate(straightLine(), tp, SimConfig{});
    EXPECT_TRUE(r.finished) << c;
    EXPECT_LT(r.settle, 3.0) << c;
    EXPECT_LT(std::abs(r.cte.back()), 0.01) << c;
  }
}

TEST(Tracker, LqrTracksTheFigureEightWithinOneCentimetre) {
  TrackerParams tp; tp.controller = "lqr"; tp.speed = 0.6;
  const SimResult r = simulate(makeFigureEight(1.6, 2), tp, SimConfig{});
  EXPECT_TRUE(r.finished);
  EXPECT_LT(r.rms_cte, 0.01);
}

TEST(Tracker, CommandsRespectTheAngularLimitAndStopAtTheGoal) {
  TrackerParams tp; tp.controller = "pure_pursuit"; tp.max_w = 0.8;
  TrackerCore t(tp);
  t.setPath(makeRoundedLoop());
  Unicycle robot({0, 0.3, 0});
  bool done = false;
  for (int k = 0; k < 20000 && !done; ++k) {
    const Command c = t.update(robot.pose());
    EXPECT_LE(std::abs(c.w), 0.8 + 1e-12);
    done = c.done;
    for (int s = 0; s < 5; ++s) robot.step(c.v, c.w, tp.dt / 5);
  }
  EXPECT_TRUE(done);
  const Command after = t.update(robot.pose());
  EXPECT_TRUE(after.done);
  EXPECT_EQ(after.v, 0.0);
  EXPECT_EQ(after.w, 0.0);
}

TEST(Sim, NoiseIsDeterministicForASeed) {
  Noise a(7), b(7);
  for (int k = 0; k < 100; ++k) EXPECT_EQ(a.gauss(1.0), b.gauss(1.0));
}
