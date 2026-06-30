// patrol_commander.cpp
// ---------------------------------------------------------------------------
// Session 4 — the "autonomous workflow" control node.
//
// A C++ Nav2 action client. It reads a list of waypoints from parameters and
// sends them, one after another, to Nav2's `navigate_to_pose` action. When the
// robot reaches a goal (or Nav2's recovery behaviors give up), it advances to
// the next waypoint and loops forever — a simple patrol / inspection routine.
//
// This is the piece that ties the whole track together: SLAM (Session 2) built
// the map, Nav2 plans + drives + recovers on that map, and THIS node is the
// high-level "mission" brain that commands it, written in C++.
//
// Concepts demonstrated: rclcpp_action client, goal/feedback/result callbacks,
// parameters (a list of [x, y, yaw] waypoints), and a small async state machine.
// ---------------------------------------------------------------------------
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct Waypoint { double x, y, yaw; };

class PatrolCommander : public rclcpp::Node
{
public:
  PatrolCommander() : Node("patrol_commander")
  {
    loop_ = declare_parameter<bool>("loop", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    // Waypoints arrive as a flat [x0,y0,yaw0, x1,y1,yaw1, ...] list — the
    // simplest type ROS 2 parameters support for a variable-length sequence.
    auto flat = declare_parameter<std::vector<double>>(
      "waypoints", {2.0, -2.0, 0.0,  2.0, 2.0, 1.57,  -2.0, 2.0, 3.14});
    for (size_t i = 0; i + 3 <= flat.size(); i += 3) {
      waypoints_.push_back({flat[i], flat[i + 1], flat[i + 2]});
    }
    if (waypoints_.empty()) {
      RCLCPP_FATAL(get_logger(), "No valid waypoints (need multiples of 3).");
      throw std::runtime_error("no waypoints");
    }

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    RCLCPP_INFO(get_logger(), "patrol_commander: %zu waypoints, loop=%s",
                waypoints_.size(), loop_ ? "true" : "false");

    // Kick things off once Nav2's action server is available.
    startup_timer_ = create_wall_timer(
      1s, std::bind(&PatrolCommander::wait_for_server, this));
  }

private:
  void wait_for_server()
  {
    if (!client_->action_server_is_ready()) {
      RCLCPP_INFO(get_logger(), "Waiting for Nav2 'navigate_to_pose' action server...");
      return;
    }
    startup_timer_->cancel();
    RCLCPP_INFO(get_logger(), "Nav2 is ready — starting patrol.");
    send_next_goal();
  }

  void send_next_goal()
  {
    const Waypoint & wp = waypoints_[current_];

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = frame_id_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = wp.x;
    goal.pose.pose.position.y = wp.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, wp.yaw);
    goal.pose.pose.orientation.x = q.x();
    goal.pose.pose.orientation.y = q.y();
    goal.pose.pose.orientation.z = q.z();
    goal.pose.pose.orientation.w = q.w();

    RCLCPP_INFO(get_logger(), "[%zu/%zu] Navigating to (%.2f, %.2f, yaw=%.2f)",
                current_ + 1, waypoints_.size(), wp.x, wp.y, wp.yaw);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback =
      [this](GoalHandle::SharedPtr gh) {
        if (!gh) {
          RCLCPP_WARN(get_logger(), "Goal rejected by Nav2 — skipping waypoint.");
          advance();
        }
      };
    opts.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_DEBUG(get_logger(), "  distance remaining: %.2f m",
                     fb->distance_remaining);
      };
    opts.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "  reached waypoint %zu.", current_ + 1);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "  Nav2 ABORTED (recoveries exhausted).");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "  goal canceled.");
            break;
          default:
            RCLCPP_WARN(get_logger(), "  unknown result code.");
            break;
        }
        advance();
      };

    client_->async_send_goal(goal, opts);
  }

  void advance()
  {
    current_++;
    if (current_ >= waypoints_.size()) {
      if (!loop_) {
        RCLCPP_INFO(get_logger(), "Patrol complete. Shutting down.");
        rclcpp::shutdown();
        return;
      }
      current_ = 0;
      RCLCPP_INFO(get_logger(), "Patrol loop complete — starting over.");
    }
    // Brief pause between goals so logs are readable and costmaps settle.
    next_goal_timer_ = create_wall_timer(1s, [this]() {
      next_goal_timer_->cancel();
      send_next_goal();
    });
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr startup_timer_, next_goal_timer_;
  std::vector<Waypoint> waypoints_;
  size_t current_{0};
  bool loop_{true};
  std::string frame_id_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolCommander>());
  rclcpp::shutdown();
  return 0;
}
