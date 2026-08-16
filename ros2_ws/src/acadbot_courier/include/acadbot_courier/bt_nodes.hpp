// bt_nodes.hpp
// ---------------------------------------------------------------------------
// The one custom leaf in behavior_trees/courier.xml: GoToLocation, which sends
// a single Nav2 goal and reports whether it arrived. Everything else in that
// file -- Sequence, RetryUntilSuccessful, Fallback, Timeout, Delay -- is a
// BehaviorTree.CPP built-in.
//
// It is a StatefulActionNode, which is the whole discipline of this file.
// A SyncActionNode would have to wait for Nav2 to finish inside tick(), and
// this node runs inside an action server that holds an action client: block
// the executor there and the result callback that would unblock it can never
// be delivered. Same rule as the state machine version -- nothing waits.
//
//   onStart()    send the goal, return RUNNING
//   onRunning()  called every tick; RUNNING until a result arrives
//   onHalted()   cancel the goal -- Timeout expired, or the delivery was
//                cancelled, or the leg the tree was on has been abandoned
//
// Ticks and Nav2 callbacks both run on the one executor thread, so the flags
// the callbacks set and the tick reads need no mutex.
// ---------------------------------------------------------------------------
#ifndef ACADBOT_COURIER__BT_NODES_HPP_
#define ACADBOT_COURIER__BT_NODES_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "acadbot_courier/types.hpp"

namespace acadbot_courier
{

/// Everything the action feedback needs to say, written by the tree and read
/// by the node's feedback timer.
///
/// The tree cannot publish feedback itself: a leaf only exists while it is
/// ticking, and between attempts the tree is sitting in a Delay with no leaf
/// running at all. Feedback has to keep flowing through that gap -- the action
/// promises at least 1 Hz -- so the numbers live out here and the timer reads
/// them whatever the tree is doing.
struct LegStatus
{
  Leg leg{Leg::PICKUP};
  std::string target_location;
  std::string state{"NAVIGATING"};   ///< NAVIGATING / RETRYING / CANCELING
  float distance_remaining{0.0f};
  uint16_t nav2_recoveries{0};
  int attempt{0};                    ///< on this leg, 1-based
  int attempts_total{0};             ///< across both legs, for the result
};

/// The handful of things a GoToLocation needs from the node that owns the
/// tree. Passed once at registration and shared by every instance.
struct NavContext
{
  rclcpp::Node * node{nullptr};
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client;
  std::shared_ptr<LegStatus> status;
  std::string goal_frame{"map"};
};

/// Drive to one pose. SUCCESS if Nav2 reports arrival, FAILURE otherwise.
///
/// The node deliberately knows nothing about retries, timeouts or which leg
/// comes next -- those are the decorators wrapped around it in courier.xml.
/// It reports its leg and target only so the feedback can name them.
class GoToLocation : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoToLocation(
    const std::string & name, const BT::NodeConfig & config, const NavContext & context);

  /// `pose` is a Pose2D and not a location name because a job's targets are
  /// frozen when it is booked. Looking the name up again here would let an
  /// edit to courier.yaml move the destination of an accepted delivery.
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  void on_goal_response(const NavGoalHandle::SharedPtr & handle);
  void on_feedback(
    NavGoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void on_result(const NavGoalHandle::WrappedResult & result);

  NavContext context_;

  NavGoalHandle::SharedPtr nav_handle_;

  /// Set by the Nav2 callbacks, read by the next onRunning(). `outcome_` only
  /// means anything once `finished_` is true.
  bool finished_{false};
  bool outcome_{false};
};

/// Teach the factory about GoToLocation, handing every instance the same
/// context. Called once, before the first tree is built.
void register_courier_nodes(BT::BehaviorTreeFactory & factory, const NavContext & context);

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__BT_NODES_HPP_
