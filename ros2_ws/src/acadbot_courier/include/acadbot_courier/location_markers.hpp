// location_markers.hpp
// ---------------------------------------------------------------------------
// Draws the location table in RViz.
//
// Nothing renders YAML, so without this the configured locations are invisible
// until the robot either arrives at one or fails to. Each location gets an
// arrow at its pose -- the arrow, not a sphere, because yaw is a real part of
// the goal and Nav2 will not report success until the robot is within
// yaw_goal_tolerance of it -- and a floating label with the name.
//
// Diagnostic rather than decorative: drawn over the global costmap, a location
// sitting inside the inflation band is visibly wrong before it has cost anyone
// an aborted delivery.
// ---------------------------------------------------------------------------
#ifndef ACADBOT_COURIER__LOCATION_MARKERS_HPP_
#define ACADBOT_COURIER__LOCATION_MARKERS_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "acadbot_courier/location_table.hpp"

namespace acadbot_courier
{

/// Build the whole marker array for `table`.
///
/// `active` names the location currently being driven to, which is drawn
/// larger and in a different colour so the RViz view corroborates the feedback
/// stream. Pass an empty string when no job is running.
///
/// Markers replace on matching namespace and id, so republishing the entire
/// array whenever the target changes costs nothing and never leaves a stale
/// arrow behind.
visualization_msgs::msg::MarkerArray make_location_markers(
  const LocationTable & table,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & active = "");

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__LOCATION_MARKERS_HPP_
