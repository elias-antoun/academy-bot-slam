// types.hpp
// ---------------------------------------------------------------------------
// The vocabulary the courier is written in: a pose on the floor plan, the two
// halves of a delivery, and a booked job.
//
// Kept apart from the node so the pieces that only *describe* a delivery --
// the location table, the markers -- need not include the one that runs it.
// ---------------------------------------------------------------------------
#ifndef ACADBOT_COURIER__TYPES_HPP_
#define ACADBOT_COURIER__TYPES_HPP_

#include <string>

namespace acadbot_courier
{

/// Where to stand, and which way to face once there.
///
/// Yaw rather than a quaternion, because that is how a location is written in
/// YAML. The conversion happens once, on the way into a Nav2 goal.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A delivery is always these two legs, in this order.
enum class Leg
{
  PICKUP,
  DROPOFF,
};

/// Spelled once: this string goes into the action feedback and into the
/// result's failed_leg field, and those two must agree.
inline const char * to_string(Leg leg)
{
  return leg == Leg::PICKUP ? "pickup" : "dropoff";
}

/// The life of a job.
///
/// The terminal states are remembered rather than forgotten, so a second
/// ExecuteDelivery goal carrying an id that has already run is rejected
/// instead of quietly delivering twice.
enum class JobState
{
  BOOKED,      // accepted by the service, waiting for an action goal
  RUNNING,
  SUCCEEDED,
  FAILED,
  CANCELED,
};

inline const char * to_string(JobState state)
{
  switch (state) {
    case JobState::BOOKED:    return "BOOKED";
    case JobState::RUNNING:   return "RUNNING";
    case JobState::SUCCEEDED: return "SUCCEEDED";
    case JobState::FAILED:    return "FAILED";
    case JobState::CANCELED:  return "CANCELED";
  }
  return "UNKNOWN";
}

/// A booked delivery.
///
/// Both the names and the resolved poses are stored: the names are what the
/// requester asked for and what the feedback reports, the poses are what Nav2
/// is sent. Resolving once, at booking time, is the point -- looking the names
/// up again when the action runs would let an edit to courier.yaml move the
/// target of a job that has already been accepted.
struct Job
{
  std::string id;
  std::string pickup_name;
  std::string dropoff_name;
  Pose2D pickup;
  Pose2D dropoff;
  JobState state{JobState::BOOKED};

  const std::string & name_of(Leg leg) const
  {
    return leg == Leg::PICKUP ? pickup_name : dropoff_name;
  }

  const Pose2D & pose_of(Leg leg) const
  {
    return leg == Leg::PICKUP ? pickup : dropoff;
  }
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__TYPES_HPP_
