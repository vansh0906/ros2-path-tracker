// Closed-loop benchmark: Pure Pursuit vs LQR on a figure-8 and a rounded loop, with and without
// odometry noise. Same TrackerCore class the ROS 2 node uses. Writes bench_traj.csv for plot.py.
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "path_tracker/sim.hpp"

using namespace path_tracker;

int main() {
  struct Case { std::string name; Path path; double speed; double xy_noise, yaw_noise; };
  const Path fig8 = makeFigureEight(1.6, 2), loop = makeRoundedLoop(3.0, 1.5, 0.5, 2);
  const std::vector<Case> cases = {
      {"figure-8, 0.3 m/s", fig8, 0.3, 0, 0},
      {"figure-8, 0.6 m/s", fig8, 0.6, 0, 0},
      {"loop, 0.6 m/s", loop, 0.6, 0, 0},
      {"figure-8, 0.6 m/s, noisy odom", fig8, 0.6, 0.01, 0.01},
  };
  std::printf("Robot: diff-drive, |w| <= 2 rad/s, 0.15 s velocity lag; control at 50 Hz; starts 0.2 m off the path\n");
  std::printf("Figure-8: %.1f m per lap (max curvature %.2f 1/m), 2 laps. Loop: %.1f m per lap, 2 laps.\n\n",
              fig8.length() / 2, [&] { double k = 0; for (std::size_t i = 0; i < fig8.size(); ++i) k = std::max(k, std::abs(fig8.kappa(i))); return k; }(),
              loop.length() / 2);
  std::printf("%-31s %-19s %8s %9s %9s %7s %10s\n", "scenario", "controller", "settle", "RMS CTE", "max CTE", "w rate", "lap time");
  std::ofstream csv("bench_traj.csv");
  csv << "case,controller,t,x,y,cte\n";
  for (const auto& c : cases) {
    for (const std::string ctrl : {"pure_pursuit", "lqr"}) {
      TrackerParams tp; tp.controller = ctrl; tp.speed = c.speed;
      SimConfig sc; sc.odom_xy_noise = c.xy_noise; sc.odom_yaw_noise = c.yaw_noise;
      const SimResult r = simulate(c.path, tp, sc);
      const std::string label = ctrl == "lqr" ? "LQR + feed-forward" : "Pure Pursuit";
      std::printf("%-31s %-19s %6.2f s %6.1f cm %6.1f cm %6.2f %8.1f s%s\n", c.name.c_str(), label.c_str(), r.settle,
                  r.rms_cte * 100, r.max_cte * 100, r.mean_w_rate, r.time, r.finished ? "" : "  (did not finish)");
      if (c.name == "figure-8, 0.6 m/s")
        for (std::size_t i = 0; i < r.t.size(); i += 2)
          csv << "fig8," << ctrl << ',' << r.t[i] << ',' << r.x[i] << ',' << r.y[i] << ',' << r.cte[i] << '\n';
    }
    std::printf("\n");
  }
  std::printf("settle = time to get within 3 cm of the path; errors are scored after the first 3 s\n");
  std::printf("w rate = mean |change in commanded angular velocity|, rad/s^2 (lower = smoother); lap time = both laps\n");
  std::printf("wrote bench_traj.csv (plot with: python plot.py)\n");
  return 0;
}
