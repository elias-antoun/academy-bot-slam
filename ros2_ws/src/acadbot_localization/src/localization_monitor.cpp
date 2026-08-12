// localization_monitor.cpp
// ---------------------------------------------------------------------------
// Session 2 — "is AMCL actually localised, or is it just running?"
//
// SKELETON ONLY. This compiles and runs so the rest of the package can be
// built and launched, but it does not do the job yet.
//
// OWNER: person 2 (the C++ node).  What it has to do:
//   * subscribe to /amcl_pose  (geometry_msgs/msg/PoseWithCovarianceStamped)
//   * on a timer, log one line: x, y, yaw, and AMCL's uncertainty
//   * uncertainty = sqrt(max(covariance[0], covariance[7])) in metres
//       covariance is a row-major 6x6 array: index 0 = variance in x,
//       index 7 = variance in y
//   * parameters from config/localization_monitor.yaml:
//       report_period    (s, default 1.0)  — seconds between log lines
//       converged_sigma  (m, default 0.25) — CONVERGED below it, SEARCHING above
//   * before an initial pose is set, AMCL publishes NOTHING at all. Log a
//     warning saying so rather than printing zeros — that warning is the most
//     useful line this node produces.
//
// Keep it small: a subscriber, a timer, two parameters and a log line.
// ---------------------------------------------------------------------------
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor() : Node("localization_monitor"), has_received_pose(_false_)
  {
    this->declare_parameter<double>("report_period", 1.0);
    this->declare_parameter<double>("converged_sigma", 0.25);

    double report_period = this->get_parameter("report_period").as_double();
    converged_sigma_ = this->get_parameter("converged_sigma").as_double();

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&LocalizationMonitor::pose_callback, this, std::placeholders::_1));

    auto timer_period = std::chrono::duration<double>(report_period);
    timer_ = this->create_wall_timer(
      timer_period, std::bind(&LocalizationMonitor::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Localization Monitor node started.");
  }
private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_pose_;
  bool has_received_pose_;
  double converged_sigma_;

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    last_pose_ = msg;
    has_received_pose_ = true;
  }

  void timer_callback()
  {
    if (!has_received_pose_)
    {
      RCLCPP_WARN(this->get_logger(), "No AMCL pose received yet.");
      return;
    }

    double x = last_pose_->pose.pose.position.x;
    double y = last_pose_->pose.pose.position.y;

    tf2::Quaternion q(
      last_pose_->pose.pose.orientation.x,
      last_pose_->pose.pose.orientation.y,
      last_pose_->pose.pose.orientation.z,
      last_pose_->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Row-major 6x6 covariance matrix: Index 0 is var(x), Index 7 is var(y)
    double var_x = last_pose_->pose.covariance[0];
    double var_y = last_pose_->pose.covariance[7];
    double max_var = std::max(var_x, var_y);
    double sigma = (max_var > 0.0) ? std::sqrt(max_var) : 0.0;

    // Determine convergence status
    std::string status = (sigma <= converged_sigma_) ? "CONVERGED" : "SEARCHING";

    RCLCPP_INFO(this->get_logger(), "x: %.3f, y: %.3f, yaw: %.3f, uncertainty: %.3f m (%s)",
                x, y, yaw, sigma, status.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
