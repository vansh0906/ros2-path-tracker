// Path-tracking node: follows the nav_msgs/Path on /plan using /odom, publishes /cmd_vel.
// Works with the simulated robot or any real ROS 2 differential-drive base (TurtleBot, ROSMASTER, ...).
//   controller:=lqr | pure_pursuit
// Publishes /tracking_error (std_msgs/Float64), logs a CSV, and prints a summary at the goal.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_tracker/ros_utils.hpp"
#include "path_tracker/tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace pt = path_tracker;

static pt::TrackerParams declareParams(rclcpp::Node& n) {
  pt::TrackerParams p;
  p.controller = n.declare_parameter<std::string>("controller", p.controller);
  p.speed = n.declare_parameter<double>("speed", p.speed);
  p.max_w = n.declare_parameter<double>("max_w", p.max_w);
  p.max_lateral_acc = n.declare_parameter<double>("max_lateral_acc", p.max_lateral_acc);
  p.goal_tolerance = n.declare_parameter<double>("goal_tolerance", p.goal_tolerance);
  p.slow_down_dist = n.declare_parameter<double>("slow_down_dist", p.slow_down_dist);
  p.dt = 1.0 / n.declare_parameter<double>("control_rate", 50.0);
  p.pp_gain = n.declare_parameter<double>("pp_gain", p.pp_gain);
  p.pp_min_lookahead = n.declare_parameter<double>("pp_min_lookahead", p.pp_min_lookahead);
  p.lqr_q_e = n.declare_parameter<double>("lqr_q_e", p.lqr_q_e);
  p.lqr_q_th = n.declare_parameter<double>("lqr_q_th", p.lqr_q_th);
  p.lqr_r = n.declare_parameter<double>("lqr_r", p.lqr_r);
  p.lqr_preview = n.declare_parameter<double>("lqr_preview", p.lqr_preview);
  return p;
}

class TrackerNode : public rclcpp::Node {
 public:
  TrackerNode() : Node("tracker"), params_(declareParams(*this)), core_(params_) {
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "odom");
    const std::string cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    const std::string truth_topic = declare_parameter<std::string>("ground_truth_topic", "");
    const std::string csv_path = declare_parameter<std::string>("csv_path", "");
    exit_on_goal_ = declare_parameter<bool>("exit_on_goal", true);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    err_pub_ = create_publisher<std_msgs::msg::Float64>("tracking_error", 10);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "plan", rclcpp::QoS(1).reliable().transient_local(),
        [this](nav_msgs::msg::Path::ConstSharedPtr msg) { onPath(*msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          pose_ = toPose(*msg);
          last_odom_ = now();
        });
    if (!truth_topic.empty()) {
      truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          truth_topic, rclcpp::SensorDataQoS(),
          [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { truth_ = toPose(*msg); });
    }
    if (!csv_path.empty()) {
      csv_.open(csv_path);
      if (csv_) csv_ << "t,x,y,yaw,cte,true_cte,v,w\n";
      else RCLCPP_ERROR(get_logger(), "Cannot write %s", csv_path.c_str());
    }
    timer_ = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(params_.dt * 1e9)),
                               [this]() { onTimer(); });
    RCLCPP_INFO(get_logger(), "%s tracker at %.2f m/s, %.0f Hz. Waiting for /plan and %s...",
                core_.controller().name().c_str(), params_.speed, 1.0 / params_.dt, odom_topic.c_str());
  }

 private:
  static pt::Pose2D toPose(const nav_msgs::msg::Odometry& o) {
    return {o.pose.pose.position.x, o.pose.pose.position.y, pt::yawFromQuaternion(o.pose.pose.orientation)};
  }

  void onPath(const nav_msgs::msg::Path& msg) {
    std::vector<double> xs, ys;
    double sig = 0;
    for (const auto& ps : msg.poses) {
      xs.push_back(ps.pose.position.x);
      ys.push_back(ps.pose.position.y);
      sig += 1.7 * ps.pose.position.x + 3.1 * ps.pose.position.y;
    }
    // the publisher repeats the same path every second: only a NEW path restarts tracking
    if (core_.hasPath() && xs.size() == last_path_size_ && std::abs(sig - last_path_sig_) < 1e-9) return;
    last_path_size_ = xs.size();
    last_path_sig_ = sig;
    try {
      core_.setPath(pt::Path(xs, ys));
      if (!started_) RCLCPP_INFO(get_logger(), "Got a %.1f m path with %zu poses", core_.path().length(), xs.size());
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "Ignoring path: %s", e.what());
    }
  }

  void stop() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

  void onTimer() {
    if (!core_.hasPath() || !pose_) return;
    if ((now() - last_odom_).seconds() > odom_timeout_) {   // safety: never drive blind
      stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No odometry for %.1f s: stopped", odom_timeout_);
      return;
    }
    if (!started_) { started_ = true; t0_ = now(); }
    const pt::Command c = core_.update(*pose_);
    if (c.done) { finish(); return; }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = c.v;
    cmd.angular.z = c.w;
    cmd_pub_->publish(cmd);

    std_msgs::msg::Float64 err;
    err.data = c.proj.cross_track;
    err_pub_->publish(err);

    const double t = (now() - t0_).seconds();
    double true_cte = c.proj.cross_track;
    if (truth_) true_cte = core_.path().project(truth_->x, truth_->y, truth_->yaw, c.proj.index).cross_track;
    if (t > 3.0) { sq_ += true_cte * true_cte; max_ = std::max(max_, std::abs(true_cte)); ++n_; }
    if (csv_) csv_ << t << ',' << pose_->x << ',' << pose_->y << ',' << pose_->yaw << ',' << c.proj.cross_track << ','
                   << true_cte << ',' << c.v << ',' << c.w << '\n';
  }

  void finish() {
    if (finished_) return;
    finished_ = true;
    for (int k = 0; k < 5; ++k) stop();
    const double t = (now() - t0_).seconds();
    const double rms = n_ ? std::sqrt(sq_ / static_cast<double>(n_)) : 0.0;
    RCLCPP_INFO(get_logger(), "GOAL REACHED in %.1f s | %s | RMS cross-track %.1f cm | max %.1f cm (%s, after the first 3 s)",
                t, core_.controller().name().c_str(), rms * 100, max_ * 100, truth_ ? "vs ground truth" : "vs odometry");
    if (csv_) csv_.close();
    if (exit_on_goal_) rclcpp::shutdown();
  }

  pt::TrackerParams params_;
  pt::TrackerCore core_;
  bool exit_on_goal_ = true, started_ = false, finished_ = false;
  double odom_timeout_ = 0.5, sq_ = 0, max_ = 0, last_path_sig_ = 0;
  std::size_t last_path_size_ = 0;
  long n_ = 0;
  std::optional<pt::Pose2D> pose_, truth_;
  rclcpp::Time last_odom_, t0_;
  std::ofstream csv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, truth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
