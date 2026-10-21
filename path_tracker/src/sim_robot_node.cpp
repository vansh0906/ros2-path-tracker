// A stand-in for a real differential-drive robot, so the stack runs with no Gazebo and no hardware.
// Subscribes /cmd_vel, simulates the robot (velocity lag, acceleration and turn-rate limits), and
// publishes /odom (with optional measurement noise), /ground_truth and the odom -> base_link TF.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "path_tracker/ros_utils.hpp"
#include "path_tracker/sim.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class SimRobot : public rclcpp::Node {
 public:
  SimRobot() : Node("sim_robot"), noise_(1) {
    const double rate = declare_parameter<double>("physics_rate", 200.0);
    const double odom_rate = declare_parameter<double>("odom_rate", 50.0);
    path_tracker::Pose2D start;
    start.x = declare_parameter<double>("start_x", 0.0);
    start.y = declare_parameter<double>("start_y", 0.2);
    start.yaw = declare_parameter<double>("start_yaw", 0.0);
    path_tracker::RobotLimits lim;
    lim.max_v = declare_parameter<double>("max_v", 1.0);
    lim.max_w = declare_parameter<double>("max_w", 2.0);
    lim.max_acc = declare_parameter<double>("max_acc", 1.0);
    lim.max_alpha = declare_parameter<double>("max_alpha", 4.0);
    lim.time_constant = declare_parameter<double>("time_constant", 0.15);
    xy_noise_ = declare_parameter<double>("odom_xy_noise", 0.0);
    yaw_noise_ = declare_parameter<double>("odom_yaw_noise", 0.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    noise_ = path_tracker::Noise(static_cast<std::uint32_t>(declare_parameter<int64_t>("seed", 1)));

    robot_ = path_tracker::Unicycle(start, lim);
    dt_ = 1.0 / rate;
    last_cmd_time_ = now();
    last_step_ = last_cmd_time_;

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          cmd_v_ = msg->linear.x;
          cmd_w_ = msg->angular.z;
          last_cmd_time_ = now();
        });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("ground_truth", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    physics_timer_ = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / rate)), [this]() {
      // safety: a real base stops if commands stop arriving
      const rclcpp::Time t = now();
      const bool stale = (t - last_cmd_time_).seconds() > cmd_timeout_;
      // integrate over the real elapsed time (timers jitter), capped so a hiccup can't teleport the robot
      const double dt = std::min(4 * dt_, std::max(0.0, (t - last_step_).seconds()));
      last_step_ = t;
      robot_.step(stale ? 0.0 : cmd_v_, stale ? 0.0 : cmd_w_, dt);
    });
    odom_timer_ = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / odom_rate)),
                                    [this]() { publishOdom(); });
    RCLCPP_INFO(get_logger(), "Simulated robot at (%.2f, %.2f), odom noise %.3f m / %.3f rad", start.x, start.y,
                xy_noise_, yaw_noise_);
  }

 private:
  nav_msgs::msg::Odometry makeOdom(const path_tracker::Pose2D& p, const rclcpp::Time& stamp) const {
    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = p.x;
    o.pose.pose.position.y = p.y;
    o.pose.pose.orientation = path_tracker::quaternionFromYaw(p.yaw);
    o.twist.twist.linear.x = robot_.v();
    o.twist.twist.angular.z = robot_.w();
    return o;
  }

  void publishOdom() {
    const rclcpp::Time stamp = now();
    const path_tracker::Pose2D truth = robot_.pose();
    path_tracker::Pose2D meas = truth;
    meas.x += noise_.gauss(xy_noise_);
    meas.y += noise_.gauss(xy_noise_);
    meas.yaw = path_tracker::wrapAngle(meas.yaw + noise_.gauss(yaw_noise_));
    odom_pub_->publish(makeOdom(meas, stamp));
    truth_pub_->publish(makeOdom(truth, stamp));

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = truth.x;
    tf.transform.translation.y = truth.y;
    tf.transform.rotation = path_tracker::quaternionFromYaw(truth.yaw);
    tf_->sendTransform(tf);
  }

  path_tracker::Unicycle robot_;
  path_tracker::Noise noise_;
  double dt_ = 0.005, cmd_v_ = 0, cmd_w_ = 0, cmd_timeout_ = 0.5, xy_noise_ = 0, yaw_noise_ = 0;
  std::string odom_frame_, base_frame_;
  rclcpp::Time last_cmd_time_, last_step_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, truth_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr physics_timer_, odom_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimRobot>());
  rclcpp::shutdown();
  return 0;
}
