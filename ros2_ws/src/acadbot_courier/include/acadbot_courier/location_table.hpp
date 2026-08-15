// location_table.hpp
// ---------------------------------------------------------------------------
// The name -> pose table the courier resolves delivery requests against.
//
// This is the only class that knows a location has coordinates at all: the
// service takes names, the action goal carries a job id, and the floor plan
// lives in YAML. Adding a room to the building is an edit to courier.yaml,
// not a rebuild.
//
//     locations:
//       reception: [0.60, 4.20, 0.00]     # x, y, yaw
//       lab_bench: [4.50, 4.20, 3.14]
//
// A location exists by being written down, and nothing else. The names are
// read from the parameter overrides rather than from a second list, so there
// is no list to fall out of step with the poses.
// ---------------------------------------------------------------------------
#ifndef ACADBOT_COURIER__LOCATION_TABLE_HPP_
#define ACADBOT_COURIER__LOCATION_TABLE_HPP_

#include <map>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "acadbot_courier/types.hpp"

namespace acadbot_courier
{

class LocationTable
{
public:
  /// Declare and read every location parameter on `node`.
  ///
  /// Throws std::runtime_error if no location is configured, or if one is not
  /// a list of exactly three numbers. Both are configuration mistakes, and a
  /// configuration mistake should stop the node at startup rather than
  /// surface as a failed delivery mid-demo.
  static LocationTable from_parameters(rclcpp::Node & node);

  /// The pose for `name`, or nothing if no such location is configured.
  std::optional<Pose2D> find(const std::string & name) const;

  /// The configured names as "lab_bench, reception, storage". Goes into the
  /// rejection reason, so a mistyped location tells the requester what they
  /// could have said instead.
  std::string known_names() const;

  /// Every location, for drawing the markers.
  const std::map<std::string, Pose2D> & all() const { return poses_; }

  std::size_t size() const { return poses_.size(); }

private:
  std::map<std::string, Pose2D> poses_;
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__LOCATION_TABLE_HPP_
