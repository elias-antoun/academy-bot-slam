// localization_monitor.cpp
// ---------------------------------------------------------------------------
// Session 2 homework — is AMCL actually localised, or just running?
//
// RViz shows a cloud of particles that visibly tightens as the robot drives.
// This node puts a number on that. It subscribes to /amcl_pose, pulls the
// position variance out of the covariance matrix, and reports the standard
// deviation once a second alongside a CONVERGED / SEARCHING verdict.
//
// Before an initial pose is set, AMCL publishes nothing at all -- so "waiting
// for /amcl_pose" is itself the answer to why map->odom does not exist yet.
// ---------------------------------------------------------------------------
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;
using PoseCov = geometry_msgs::msg::PoseWithCovarianceStamped;

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor() : Node("localization_monitor")
  {
    report_period_    = declare_parameter<double>("report_period", 1.0);
    converged_sigma_  = declare_parameter<double>("converged_sigma", 0.25);

    pose_sub_ = create_subscription<PoseCov>(
      "amcl_pose", 10,
      [this](const PoseCov::SharedPtr msg) { latest_ = *msg; have_pose_ = true; });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(report_period_),
      std::bind(&LocalizationMonitor::report, this));

    RCLCPP_INFO(get_logger(),
      "localization_monitor: reporting every %.1fs, converged below %.2f m.",
      report_period_, converged_sigma_);
  }

private:
  void report()
  {
    if (!have_pose_) {
      RCLCPP_WARN(get_logger(),
        "No /amcl_pose yet — set the initial pose in RViz (2D Pose Estimate).");
      return;
    }

    const auto & p = latest_.pose.pose.position;
    const auto & q = latest_.pose.pose.orientation;
    const auto & c = latest_.pose.covariance;

    // Yaw from the quaternion, without pulling in tf2 for two lines of algebra.
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    // Covariance is row-major 6x6: index 0 is var(x), index 7 is var(y).
    // Clamp at zero: AMCL's very first published pose can carry a tiny
    // negative variance, and sqrt() of that is NaN, which compares false
    // against every threshold and silently reads as "never converged".
    const double sigma = std::sqrt(std::max({c[0], c[7], 0.0}));

    RCLCPP_INFO(get_logger(),
      "x=%6.2f  y=%6.2f  yaw=%6.1f deg   sigma=%.3f m   %s",
      p.x, p.y, yaw * 180.0 / M_PI, sigma,
      sigma < converged_sigma_ ? "CONVERGED" : "SEARCHING");
  }

  rclcpp::Subscription<PoseCov>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  PoseCov latest_;
  bool have_pose_{false};
  double report_period_, converged_sigma_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
