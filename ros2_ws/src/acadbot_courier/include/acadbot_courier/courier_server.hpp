// courier_server.hpp
// ---------------------------------------------------------------------------
// The courier node: a booking desk, a delivery runner, and a Nav2 client.
//
//   RequestDelivery (service)  book a job, get a job_id back immediately
//   ExecuteDelivery (action)   run that job, streaming progress, cancellable
//   navigate_to_pose (client)  the actual driving, which is Nav2's problem
//
// The node is an action server and an action client at the same time. That
// nesting deadlocks the moment anything blocks, so nothing here ever does:
// no spin_until_future_complete, no worker thread, no waiting on a future.
// Every step is a callback or a timer, which is why a single-threaded
// executor is enough and none of the state below needs a mutex.
//
// One job runs at a time. A second ExecuteDelivery goal is rejected while one
// is in flight, so there is exactly one Nav2 goal handle, one leg and one
// phase to track -- no collections, no bookkeeping.
// ---------------------------------------------------------------------------
#ifndef ACADBOT_COURIER__COURIER_SERVER_HPP_
#define ACADBOT_COURIER__COURIER_SERVER_HPP_

#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "acadbot_courier_msgs/action/execute_delivery.hpp"
#include "acadbot_courier_msgs/srv/request_delivery.hpp"

#include "acadbot_courier/location_table.hpp"
#include "acadbot_courier/types.hpp"

namespace acadbot_courier
{

class CourierServer : public rclcpp::Node
{
public:
  CourierServer();

private:
  using RequestDelivery = acadbot_courier_msgs::srv::RequestDelivery;
  using ExecuteDelivery = acadbot_courier_msgs::action::ExecuteDelivery;
  using ExecuteGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteDelivery>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /// Where the current leg is, within the leg. Mirrors the `state` field the
  /// action feedback publishes.
  enum class Phase
  {
    NAVIGATING,
    RETRY_WAIT,
    CANCELING,
  };

  /// Why a Nav2 goal is being cancelled.
  ///
  /// A CANCELED result from Nav2 is ambiguous on its own -- it can mean the
  /// client cancelled the delivery, that our own leg timeout gave up on this
  /// attempt, or that Nav2 abandoned the goal unilaterally. Those three want
  /// three different outcomes, so the reason is recorded when the cancel is
  /// issued rather than guessed at when the result arrives. NONE at that
  /// moment means Nav2 did it on its own, which is a failure.
  enum class CancelReason
  {
    NONE,
    CLIENT_REQUEST,
    LEG_TIMEOUT,
  };

  // ---- booking (service) ----------------------------------------------
  void handle_request(
    const std::shared_ptr<RequestDelivery::Request> request,
    std::shared_ptr<RequestDelivery::Response> response);

  // ---- execution (action server) ---------------------------------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteDelivery::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ExecuteGoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<ExecuteGoalHandle> goal_handle);

  // ---- the state machine ------------------------------------------------
  void send_leg_goal();
  void on_nav_goal_response(const NavGoalHandle::SharedPtr & handle);
  void on_nav_feedback(
    NavGoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void on_nav_result(const NavGoalHandle::WrappedResult & result);
  void on_leg_timeout();
  void on_retry_elapsed();

  /// One attempt on the current leg has failed. Retries if any remain,
  /// otherwise ends the job as a failure naming this leg.
  void leg_failed(const std::string & why);

  void finish_succeeded();
  void finish_failed(const std::string & message);
  void finish_canceled();
  void reset_to_idle();

  // ---- output -----------------------------------------------------------
  void publish_feedback();
  void publish_markers();

  // ---- helpers ----------------------------------------------------------
  geometry_msgs::msg::PoseStamped to_goal_pose(const Pose2D & pose) const;
  Job & current_job();
  static const char * to_string(Phase phase);

  // ---- configuration (all from YAML) ------------------------------------
  std::string goal_frame_;
  int max_retries_{2};          // per leg, not per job
  double retry_delay_{3.0};     // s, between attempts on the same leg
  double feedback_period_{0.5}; // s; the action promises at least 1 Hz
  double leg_timeout_{120.0};   // s, before an attempt is abandoned

  LocationTable locations_;

  // ---- interfaces -------------------------------------------------------
  rclcpp::Service<RequestDelivery>::SharedPtr request_service_;
  rclcpp_action::Server<ExecuteDelivery>::SharedPtr delivery_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr leg_timeout_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  // ---- jobs -------------------------------------------------------------
  /// Every job ever booked, including finished ones: a replayed goal for an
  /// id that has already run must be rejected, not quietly delivered twice.
  /// Grows for the life of the process, which is fine at demo scale.
  std::map<std::string, Job> jobs_;
  unsigned int next_job_number_{0};

  // ---- the one job in flight --------------------------------------------
  std::shared_ptr<ExecuteGoalHandle> execute_handle_;
  NavGoalHandle::SharedPtr nav_handle_;
  std::string active_job_id_;
  Leg leg_{Leg::PICKUP};
  Phase phase_{Phase::NAVIGATING};
  CancelReason cancel_reason_{CancelReason::NONE};
  int attempt_{0};              // on the current leg, 1-based once sent
  int attempts_total_{0};       // across both legs, reported in the result

  // Latest numbers from Nav2, republished on our own timer so that a stalled
  // Nav2 shows up as a frozen distance rather than as silence.
  float last_distance_{0.0f};
  uint16_t last_recoveries_{0};
};

}  // namespace acadbot_courier

#endif  // ACADBOT_COURIER__COURIER_SERVER_HPP_
