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
//       names: ["reception", "lab_bench"]
//       reception:
//         x: 0.60
//         y: 4.20
//         yaw: 0.00
//
// The names are listed separately rather than discovered because ROS 2
// parameters have no map type -- that list is what tells us which groups to
// go looking for.
// ---------------------------------------------------------------------------
#ifndef ACADBOT_COURIER__LOCATION_TABLE_HPP_
#define ACADBOT_COURIER__LOCATION_TABLE_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "acadbot_courier/types.hpp"

namespace acadbot_courier
{

class LocationTable
{
public:
  /// Declare and read every location parameter on `node`.
  ///
  /// Throws std::runtime_error if the table is absent, empty, names the same
  /// location twice, or lists a name with no pose behind it. All four are
  /// configuration mistakes, and a configuration mistake should stop the node
  /// at startup rather than surface as a failed delivery mid-demo.
  static LocationTable from_parameters(rclcpp::Node & node);

  /// The pose for `name`, or nothing if no such location is configured.
  std::optional<Pose2D> find(const std::string & name) const;

  /// The configured names as "reception, lab_bench, storage", in the order
  /// they were declared. Goes into the rejection reason, so a mistyped
  /// location tells the requester what they could have said instead.
  std::string known_names() const;

  /// Every location, for drawing the markers.
  const std::map<std::string, Pose2D> & all() const { return poses_; }

  std::size_t size() const { return names_.size(); }

private:
  std::vector<std::string> names_;          // declaration order, for messages
  std::map<std::string, Pose2D> poses_;     // lookup
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__LOCATION_TABLE_HPP_
