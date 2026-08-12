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

class LocalizationMonitor : public rclcpp::Node
{
public:
  LocalizationMonitor() : Node("localization_monitor")
  {
    RCLCPP_WARN(get_logger(),
      "localization_monitor is a SKELETON - it does not read /amcl_pose yet.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
