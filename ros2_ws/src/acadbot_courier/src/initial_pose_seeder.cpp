// initial_pose_seeder.cpp
// ---------------------------------------------------------------------------
// Tells AMCL where the robot starts, so the demo needs no mouse.
//
// AMCL is handed a finished map and asked where the robot is in it. It refuses
// to guess -- nav2_params.yaml sets no initial pose -- so it publishes nothing
// until it is told, and nothing downstream survives that: without map->odom the
// Nav2 costmaps cannot configure, the lifecycle manager times out, and every
// goal afterwards comes back rejected. In Sessions 3 and 4 a human clicks 2D
// Pose Estimate in RViz. "One command brings it up" means doing that without
// the click.
//
// Publishing once on a timer does not work reliably. AMCL looks up
// base_footprint->odom around the pose's timestamp, and early in a run its TF
// buffer holds a fraction of a second, so the pose lands outside it:
//
//   Failed to transform initial pose in time (extrapolation into the past.
//   Requested time 11.580 but the earliest data is at time 11.800)
//
// The window opens as TF matures, but *when* depends on the machine. Rather
// than guess a delay, this node publishes and then checks whether it worked,
// and repeats until map->odom actually exists.
//
// It will not touch a robot that is already localized: if map->odom is present
// on the first check, someone (or something) has already set a pose, and
// re-seeding would throw away a good estimate and replace it with a guess.
// ---------------------------------------------------------------------------
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace acadbot_courier
{

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

class InitialPoseSeeder : public rclcpp::Node
{
public:
  InitialPoseSeeder()
  : Node("initial_pose_seeder")
  {
    x_ = declare_parameter<double>("initial_x", 0.0);
    y_ = declare_parameter<double>("initial_y", 0.0);
    yaw_ = declare_parameter<double>("initial_yaw", 0.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    check_period_ = declare_parameter<double>("check_period", 1.0);
    confirmations_ = declare_parameter<int>("confirmations", 3);
    min_scans_ = declare_parameter<int>("min_scans", 5);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    give_up_after_ = declare_parameter<double>("give_up_after", 90.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    pose_pub_ = create_publisher<PoseWithCovarianceStamped>("/initialpose", 10);

    // AMCL only publishes map->odom off the back of a filter update, and a
    // filter update needs a scan. Seed before the laser is running and it
    // accepts the pose, emits a transform briefly, and then goes quiet.
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr) {++scans_seen_;});

    started_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(check_period_),
      std::bind(&InitialPoseSeeder::tick, this));

    RCLCPP_INFO(get_logger(),
      "seeding AMCL at %s (%.2f, %.2f, yaw %.2f) until map->odom exists",
      map_frame_.c_str(), x_, y_, yaw_);
  }

private:
  /// Is AMCL *still* publishing map->odom?
  ///
  /// Three weaker versions of this check were wrong in the same way. Asking
  /// whether the transform exists fails because tf2 caches for ten seconds
  /// and TimePointZero returns the latest available, so a transform AMCL
  /// emitted once and abandoned keeps answering yes. Asking how old it is
  /// fails too: the age is measured in simulated time, which barely advances
  /// while Gazebo is starting -- precisely the window this has to police.
  ///
  /// So ask neither. A stamp that keeps changing means AMCL is still
  /// publishing; a stamp frozen at the value it had last second means it has
  /// stopped. That holds whatever the clock is doing.
  bool transform_advancing()
  {
    try {
      const auto tf =
        tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero);
      const rclcpp::Time stamp(tf.header.stamp);
      const bool advancing = have_stamp_ && stamp > last_stamp_;
      last_stamp_ = stamp;
      have_stamp_ = true;
      return advancing;
    } catch (const tf2::TransformException &) {
      have_stamp_ = false;
      return false;
    }
  }

  /// AMCL needs this to place the pose; before it exists there is no point
  /// publishing at all.
  bool odometry_alive()
  {
    std::string ignored;
    return tf_buffer_->canTransform(
      odom_frame_, base_frame_, tf2::TimePointZero, &ignored);
  }

  void finish(const char * why)
  {
    RCLCPP_INFO(get_logger(), "%s", why);
    timer_->cancel();
    rclcpp::shutdown();
  }

  void tick()
  {
    if (transform_advancing()) {
      if (attempts_ == 0) {
        finish("map->odom is already advancing: something has set a pose "
               "already, so nothing to do.");
        return;
      }
      // Confirmed more than once on purpose. A pose accepted before AMCL is
      // really running produces a transform that appears and then stops;
      // one check of a thing that flickers is not a check.
      if (++confirmed_ >= confirmations_) {
        RCLCPP_INFO(get_logger(),
          "map->odom advancing for %d checks after %d attempt(s); localized.",
          confirmed_, attempts_);
        finish("done.");
      }
      return;
    }
    confirmed_ = 0;

    if ((now() - started_).seconds() > give_up_after_) {
      RCLCPP_ERROR(get_logger(),
        "gave up after %.0fs: AMCL never published map->odom. Set the pose by "
        "hand with RViz's 2D Pose Estimate.", give_up_after_);
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }

    if (!odometry_alive()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "waiting for %s->%s before seeding...",
        odom_frame_.c_str(), base_frame_.c_str());
      return;
    }

    if (scans_seen_ < min_scans_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "waiting for %s to stream (%d/%d scans): AMCL cannot update its "
        "filter, and so cannot hold map->odom, until it is seeing the world.",
        scan_topic_.c_str(), scans_seen_, min_scans_);
      return;
    }

    publish_pose();
  }

  void publish_pose()
  {
    PoseWithCovarianceStamped msg;
    msg.header.frame_id = map_frame_;
    // Left at zero on purpose. tf2 reads a zero stamp as "the latest
    // available", which is the one lookup that cannot lose a race against a
    // buffer that is still filling.
    msg.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    msg.pose.pose.orientation = tf2::toMsg(q);

    // The spread RViz's 2D Pose Estimate uses: half a metre and about 15
    // degrees of doubt, which is roughly how well anyone knows where a robot
    // is by pointing at it.
    msg.pose.covariance[0] = 0.25;    // var(x)
    msg.pose.covariance[7] = 0.25;    // var(y)
    msg.pose.covariance[35] = 0.068;  // var(yaw)

    pose_pub_->publish(msg);
    ++attempts_;
    RCLCPP_INFO(get_logger(), "published initial pose (attempt %d)", attempts_);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double x_, y_, yaw_;
  std::string map_frame_, odom_frame_, base_frame_;
  double check_period_, give_up_after_;
  std::string scan_topic_;
  int confirmations_{3};
  int min_scans_{5};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool have_stamp_{false};
  rclcpp::Time started_;
  int attempts_{0};
  int confirmed_{0};
  int scans_seen_{0};
};

}  // namespace acadbot_courier

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<acadbot_courier::InitialPoseSeeder>());
  rclcpp::shutdown();
  return 0;
}
