// Fits a Catmull-Rom spline through the local waypoint window.
// Curvature and acceleration limits define the trajectory speed profile.

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <planning/msg/trajectory.hpp>
#include <planning/msg/trajectory_point.hpp>

#include "math_utils.h"
#include "spline_utils.h"

class TrajectoryPlannerNode : public rclcpp::Node {
public:
  TrajectoryPlannerNode() : Node("trajectory_planner") {
    // Input topics.
    declare_parameter<std::string>("pose_topic", "/OurCar/CoM/pose");
    declare_parameter<std::string>("twist_topic", "/OurCar/CoM/twist");
    declare_parameter<std::string>("local_path_topic", "/planning/local_path");

    // Dynamic limits used by the velocity profile.
    declare_parameter<double>("max_speed", 4.0);
    declare_parameter<double>("max_lateral_accel", 1.5);
    declare_parameter<double>("max_longitudinal_accel", 2.0);

    // Normal deceleration limit.
    declare_parameter<double>("max_longitudinal_decel", 1.0);

    // Measured terminal braking limit.
    declare_parameter<double>("terminal_decel", 0.3);

    // Sampling and runtime settings.
    declare_parameter<int>("local_path_capacity", 15);
    declare_parameter<double>("sample_spacing", 0.5);
    declare_parameter<double>("planning_rate_hz", 10.0);
    declare_parameter<double>("input_timeout", 0.5);

    max_speed_ = get_parameter("max_speed").as_double();
    max_lateral_accel_ = get_parameter("max_lateral_accel").as_double();
    max_longitudinal_accel_ = get_parameter("max_longitudinal_accel").as_double();
    max_longitudinal_decel_ = get_parameter("max_longitudinal_decel").as_double();
    terminal_decel_ = get_parameter("terminal_decel").as_double();
    local_path_capacity_ =
        std::max(2, static_cast<int>(get_parameter("local_path_capacity").as_int()));
    sample_spacing_ = get_parameter("sample_spacing").as_double();
    input_timeout_ = get_parameter("input_timeout").as_double();

    trajectory_pub_ = create_publisher<planning::msg::Trajectory>(
        "/planning/trajectory", rclcpp::QoS(1));
    path_viz_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/planning/trajectory_path", rclcpp::QoS(1));

    const auto pose_topic = get_parameter("pose_topic").as_string();
    const auto twist_topic = get_parameter("twist_topic").as_string();
    const auto local_path_topic = get_parameter("local_path_topic").as_string();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_pose_ = msg;
          last_pose_time_ = now();
        });

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        twist_topic,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_twist_ = msg;
          last_twist_time_ = now();
        });

    local_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        local_path_topic,
        rclcpp::QoS(1),
        [this](const nav_msgs::msg::Path::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          latest_local_path_ = msg;
          last_local_path_time_ = now();
        });

    const double rate_hz = get_parameter("planning_rate_hz").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz)),
                                std::bind(&TrajectoryPlannerNode::Replan, this));
  }

private:
  // Build and publish a fresh trajectory from the latest consistent snapshot.
  void Replan() {
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
    geometry_msgs::msg::TwistStamped::ConstSharedPtr twist;
    nav_msgs::msg::Path::ConstSharedPtr local_path;
    rclcpp::Time last_pose_time;
    rclcpp::Time last_twist_time;
    rclcpp::Time last_local_path_time;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pose = latest_pose_;
      twist = latest_twist_;
      local_path = latest_local_path_;
      last_pose_time = last_pose_time_;
      last_twist_time = last_twist_time_;
      last_local_path_time = last_local_path_time_;
    }

    const rclcpp::Time t = now();
    const bool input_stale =
        !pose || !twist || !local_path ||
        (t - last_pose_time).seconds() > input_timeout_ ||
        (t - last_twist_time).seconds() > input_timeout_ ||
        (t - last_local_path_time).seconds() > input_timeout_;
    if (input_stale || local_path->poses.empty()) {
      planning::msg::Trajectory empty;
      empty.header.stamp = t;
      empty.header.frame_id = "world";
      trajectory_pub_->publish(empty);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Planner input timeout/empty path; publishing an empty stop trajectory");
      return;
    }

    // Start at the live ego position, then follow the selected waypoint window.
    std::vector<planning::Point2D> raw_points;
    raw_points.reserve(local_path->poses.size() + 1);
    raw_points.push_back({pose->pose.position.x, pose->pose.position.y});

    for (const auto &p : local_path->poses) {
      raw_points.push_back({p.pose.position.x, p.pose.position.y});
    }

    const auto points = planning::DedupPoints(raw_points);
    if (points.size() < 2) {
      return;  // car is already at (or past) the only remaining waypoint
    }

    auto samples = planning::SampleCatmullRom(points, sample_spacing_);
    if (samples.empty()) {
      return;
    }

    const double current_speed =
        std::hypot(twist->twist.linear.x, twist->twist.linear.y);

    // A short local window means the final waypoint is visible.
    const bool terminal_goal_visible =
        local_path->poses.size() < static_cast<size_t>(local_path_capacity_);
    const std::vector<double> velocities =
        ComputeVelocityProfile(samples, current_speed, terminal_goal_visible);
    const std::vector<double> times = ComputeTimeProfile(samples, velocities);

    PublishTrajectory(pose->header.stamp, samples, velocities, times);
  }

  // Curvature-based cap, then forward/backward acceleration-limited smoothing.
  std::vector<double> ComputeVelocityProfile(
      const std::vector<planning::SplineSample> &samples,
      double current_speed,
      bool stop_at_end) const {
    const size_t n = samples.size();
    std::vector<double> v(n);

    for (size_t i = 0; i < n; ++i) {
      const double abs_k = std::max(std::fabs(samples[i].curvature), 1e-6);
      v[i] = std::min(max_speed_, std::sqrt(max_lateral_accel_ / abs_k));
    }

    v[0] = std::min(v[0], std::max(0.0, current_speed));

    for (size_t i = 1; i < n; ++i) {
      const double ds = samples[i].arc_length - samples[i - 1].arc_length;
      const double reachable_speed = std::sqrt(
          v[i - 1] * v[i - 1] + 2.0 * max_longitudinal_accel_ * ds);
      v[i] = std::min(v[i], reachable_speed);
    }

    if (stop_at_end) {
      v.back() = 0.0;
    }

    const double backward_decel =
        stop_at_end ? terminal_decel_ : max_longitudinal_decel_;

    for (size_t i = n - 1; i-- > 0;) {
      const double ds = samples[i + 1].arc_length - samples[i].arc_length;
      const double reachable_speed = std::sqrt(
          v[i + 1] * v[i + 1] + 2.0 * backward_decel * ds);
      v[i] = std::min(v[i], reachable_speed);
    }

    return v;
  }

  std::vector<double> ComputeTimeProfile(
      const std::vector<planning::SplineSample> &samples,
      const std::vector<double> &v) const {
    std::vector<double> t(samples.size(), 0.0);

    for (size_t i = 1; i < samples.size(); ++i) {
      const double ds = samples[i].arc_length - samples[i - 1].arc_length;
      const double v_avg = std::max(0.5 * (v[i - 1] + v[i]), 0.1);
      t[i] = t[i - 1] + ds / v_avg;
    }
    return t;
  }

  void PublishTrajectory(
      const rclcpp::Time &stamp,
      const std::vector<planning::SplineSample> &samples,
      const std::vector<double> &velocities,
      const std::vector<double> &times) {
    planning::msg::Trajectory trajectory;
    trajectory.header.stamp = stamp;
    trajectory.header.frame_id = "world";
    trajectory.points.reserve(samples.size());

    nav_msgs::msg::Path path_viz;
    path_viz.header = trajectory.header;
    path_viz.poses.reserve(samples.size());

    for (size_t i = 0; i < samples.size(); ++i) {
      planning::msg::TrajectoryPoint tp;
      tp.pose.position.x = samples[i].x;
      tp.pose.position.y = samples[i].y;
      tp.pose.orientation = planning::YawToQuaternion(samples[i].yaw);
      tp.velocity = static_cast<float>(velocities[i]);
      tp.curvature = static_cast<float>(samples[i].curvature);
      tp.time_from_start = static_cast<float>(times[i]);
      trajectory.points.push_back(tp);

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = trajectory.header;
      pose_stamped.pose = tp.pose;
      path_viz.poses.push_back(pose_stamped);
    }

    trajectory_pub_->publish(trajectory);
    path_viz_pub_->publish(path_viz);
  }

  // Planning configuration.
  double max_speed_{4.0};
  double max_lateral_accel_{1.5};
  double max_longitudinal_accel_{2.0};
  double max_longitudinal_decel_{1.0};
  double terminal_decel_{0.3};
  int local_path_capacity_{15};
  double sample_spacing_{0.5};
  double input_timeout_{0.5};

  // Latest synchronized-enough input snapshot.
  std::mutex state_mutex_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr latest_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr latest_twist_;
  nav_msgs::msg::Path::ConstSharedPtr latest_local_path_;
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_twist_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_local_path_time_{0, 0, RCL_ROS_TIME};

  // ROS interfaces.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
  rclcpp::Publisher<planning::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_viz_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
