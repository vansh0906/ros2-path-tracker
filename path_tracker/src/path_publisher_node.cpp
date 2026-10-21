// Publishes a test path (figure-8 or rounded loop) as nav_msgs/Path on /plan.
// With relative_to_robot:=true it waits for the first /odom message and places the path so it
// starts exactly where the robot is standing, facing the way it faces (for real robots).
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_tracker/path.hpp"
#include "path_tracker/ros_utils.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node {
 public:
  PathPublisher() : Node("path_publisher") {
    shape_ = declare_parameter<std::string>("shape", "figure8");
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    laps_ = static_cast<int>(declare_parameter<int64_t>("laps", 2));
    fig8_half_width_ = declare_parameter<double>("figure8_half_width", 1.6);
    loop_length_ = declare_parameter<double>("loop_length", 3.0);
    loop_width_ = declare_parameter<double>("loop_width", 1.5);
    loop_radius_ = declare_parameter<double>("loop_radius", 0.5);
    relative_ = declare_parameter<bool>("relative_to_robot", false);
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "odom");

    // transient_local: a tracker that starts later still receives the last path
    pub_ = create_publisher<nav_msgs::msg::Path>("plan", rclcpp::QoS(1).reliable().transient_local());

    if (relative_) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic, rclcpp::SensorDataQoS(), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            if (published_) return;
            build(msg->pose.pose.position.x, msg->pose.pose.position.y,
                  path_tracker::yawFromQuaternion(msg->pose.pose.orientation));
            RCLCPP_INFO(get_logger(), "Path placed at the robot's pose (%.2f, %.2f)", msg->pose.pose.position.x,
                        msg->pose.pose.position.y);
          });
      RCLCPP_INFO(get_logger(), "Waiting for %s to place the path at the robot...", odom_topic.c_str());
    } else {
      build(0.0, 0.0, 0.0);
    }
    // re-publish every second so RViz and late subscribers always have it
    timer_ = create_wall_timer(1s, [this]() { if (published_) pub_->publish(msg_); });
  }

 private:
  void build(double ox, double oy, double oyaw) {
    const path_tracker::Path p = shape_ == "loop"
        ? path_tracker::makeRoundedLoop(loop_length_, loop_width_, loop_radius_, laps_)
        : path_tracker::makeFigureEight(fig8_half_width_, laps_);
    if (shape_ != "loop" && shape_ != "figure8")
      RCLCPP_WARN(get_logger(), "Unknown shape '%s', using figure8", shape_.c_str());
    msg_.header.frame_id = frame_id_;
    msg_.header.stamp = now();
    msg_.poses.clear();
    const double c = std::cos(oyaw), s = std::sin(oyaw);
    for (std::size_t i = 0; i < p.size(); i += 2) {   // every 4 cm is plenty for the message
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg_.header;
      ps.pose.position.x = ox + c * p.x(i) - s * p.y(i);
      ps.pose.position.y = oy + s * p.x(i) + c * p.y(i);
      ps.pose.orientation = path_tracker::quaternionFromYaw(oyaw + p.yaw(i));
      msg_.poses.push_back(ps);
    }
    pub_->publish(msg_);
    published_ = true;
    RCLCPP_INFO(get_logger(), "Published %s path: %.1f m, %zu poses in frame '%s'", shape_.c_str(), p.length(),
                msg_.poses.size(), frame_id_.c_str());
  }

  std::string shape_, frame_id_;
  int laps_;
  double fig8_half_width_, loop_length_, loop_width_, loop_radius_;
  bool relative_, published_ = false;
  nav_msgs::msg::Path msg_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}
