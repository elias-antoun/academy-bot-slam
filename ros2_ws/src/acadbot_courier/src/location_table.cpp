#include "acadbot_courier/location_table.hpp"

#include <stdexcept>

namespace acadbot_courier
{

namespace
{
// declare_parameter<T>(name) with no default value throws this when the YAML
// supplies no override. Aliased once because it is not the name you would
// guess, and this file leans on it in two places.
using MissingParameter =
  rclcpp::exceptions::UninitializedStaticallyTypedParameterException;
}  // namespace

LocationTable LocationTable::from_parameters(rclcpp::Node & node)
{
  LocationTable table;

  // Declared without a default on purpose: a courier with no floor plan can
  // serve no request at all, so an absent list is a startup failure rather
  // than something to discover on the first delivery.
  try {
    table.names_ =
      node.declare_parameter<std::vector<std::string>>("locations.names");
  } catch (const MissingParameter &) {
    throw std::runtime_error(
      "no 'locations.names' parameter: the courier has no floor plan to work "
      "from. Launch it with a config file containing a locations block.");
  }

  if (table.names_.empty()) {
    throw std::runtime_error(
      "'locations.names' is empty: no location is configured.");
  }

  for (const auto & name : table.names_) {
    if (table.poses_.count(name) != 0) {
      throw std::runtime_error(
        "location '" + name + "' is listed twice in locations.names.");
    }

    const std::string key = "locations." + name;
    std::vector<double> xyyaw;
    try {
      xyyaw = node.declare_parameter<std::vector<double>>(key);
    } catch (const MissingParameter &) {
      throw std::runtime_error(
        "location '" + name + "' is listed in locations.names but has no "
        "[x, y, yaw] behind it.");
    }

    // Refused rather than skipped. A short list is a typo, and skipping it
    // would surface an hour later as "unknown location" pointing at the
    // service handler instead of at the missing number.
    if (xyyaw.size() != 3) {
      throw std::runtime_error(
        "location '" + name + "' needs exactly 3 numbers [x, y, yaw], got " +
        std::to_string(xyyaw.size()) + ".");
    }

    table.poses_.emplace(name, Pose2D{xyyaw[0], xyyaw[1], xyyaw[2]});
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
  for (std::size_t i = 0; i < names_.size(); ++i) {
    if (i != 0) {
      out += ", ";
    }
    out += names_[i];
  }
  return out;
}

}  // namespace acadbot_courier
