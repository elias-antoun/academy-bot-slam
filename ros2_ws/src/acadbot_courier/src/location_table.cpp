#include "acadbot_courier/location_table.hpp"

#include <stdexcept>

namespace acadbot_courier
{

LocationTable LocationTable::from_parameters(rclcpp::Node & node)
{
  LocationTable table;
  const std::string prefix = "locations.";

  // The overrides are every parameter the YAML actually supplied, whether or
  // not it has been declared. Reading them directly is what lets a location
  // exist purely by being written down: there is no second list of names to
  // keep in step, so the two cannot disagree.
  const auto & overrides =
    node.get_node_parameters_interface()->get_parameter_overrides();

  for (const auto & kv : overrides) {
    const std::string & key = kv.first;
    if (key.rfind(prefix, 0) != 0) {
      continue;
    }
    const std::string name = key.substr(prefix.size());

    std::vector<double> xyyaw;
    try {
      xyyaw = node.declare_parameter<std::vector<double>>(key);
    } catch (const rclcpp::exceptions::InvalidParameterTypeException &) {
      throw std::runtime_error(
        "location '" + name + "' must be a list of three numbers, "
        "[x, y, yaw].");
    }

    // Refused rather than skipped. A short list is a typo, and skipping it
    // would surface an hour later as "unknown location", sending you to the
    // service handler when the actual fault is a forgotten yaw.
    if (xyyaw.size() != 3) {
      throw std::runtime_error(
        "location '" + name + "' needs exactly 3 numbers [x, y, yaw], got " +
        std::to_string(xyyaw.size()) + ".");
    }

    table.poses_.emplace(name, Pose2D{xyyaw[0], xyyaw[1], xyyaw[2]});
  }

  // A courier with no floor plan can serve no request at all, so this is a
  // startup failure rather than something to discover on the first delivery.
  if (table.poses_.empty()) {
    throw std::runtime_error(
      "no locations configured: courier.yaml needs a 'locations:' block, "
      "e.g.  locations:\\n    reception: [0.60, 4.20, 0.00]");
  }

  return table;
}

std::optional<Pose2D> LocationTable::find(const std::string & name) const
{
  const auto it = poses_.find(name);
  if (it == poses_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::string LocationTable::known_names() const
{
  std::string out;
  for (const auto & kv : poses_) {
    if (!out.empty()) {
      out += ", ";
    }
    out += kv.first;
  }
  return out;
}

}  // namespace acadbot_courier
