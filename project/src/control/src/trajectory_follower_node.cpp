// Converts the gated trajectory into Unity vehicle commands.
// Pure Pursuit controls steering, while a PI controller regulates speed.
// All vehicle-model values are configurable ROS parameters.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <planning/msg/trajectory.hpp>
#include <simulation/msg/vehicle_control.hpp>

#include "math_utils.h"

class TrajectoryFollowerNode : public rclcpp::Node {
public:
  TrajectoryFollowerNode() : Node("trajectory_follower") {
    // Input topics.
    declare_parameter<std::string>("trajectory_topic", "/decision_making/trajectory");
    declare_parameter<std::string>("pose_topic", "/OurCar/CoM/pose");
    declare_parameter<std::string>("twist_topic", "/OurCar/CoM/twist");

    // Lateral-control geometry and lookahead settings.
    declare_parameter<double>("wheelbase", 2.63);
    declare_parameter<double>("max_steering_angle", 0.6);
    declare_parameter<double>("lookahead_min", 2.0);
    declare_parameter<double>("lookahead_max", 8.0);
    declare_parameter<double>("lookahead_speed_gain", 0.6);
    declare_parameter<double>("speed_preview_min_distance", 1.0);
    declare_parameter<double>("speed_preview_extra_distance", 5.0);

    // Longitudinal controller gains.
    declare_parameter<double>("speed_kp", 0.6);
    declare_parameter<double>("speed_ki", 0.1);
    declare_parameter<double>("speed_integral_limit", 2.0);
    declare_parameter<double>("accel_to_throttle_gain", 1.0);
    declare_parameter<double>("accel_to_brake_gain", 1.0);

    // Runtime safety settings.
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("command_timeout", 0.5);
    declare_parameter<double>("stop_velocity_epsilon", 0.05);

    wheelbase_ = get_parameter("wheelbase").as_double();
    max_steering_angle_ = get_parameter("max_steering_angle").as_double();
    lookahead_min_ = get_parameter("lookahead_min").as_double();
    lookahead_max_ = get_parameter("lookahead_max").as_double();
    lookahead_speed_gain_ = get_parameter("lookahead_speed_gain").as_double();
    speed_preview_min_distance_ = get_parameter("speed_preview_min_distance").as_double();
    speed_preview_extra_distance_ = get_parameter("speed_preview_extra_distance").as_double();
    speed_kp_ = get_parameter("speed_kp").as_double();
    speed_ki_ = get_parameter("speed_ki").as_double();
    speed_integral_limit_ = get_parameter("speed_integral_limit").as_double();
    accel_to_throttle_gain_ = get_parameter("accel_to_throttle_gain").as_double();
    accel_to_brake_gain_ = get_parameter("accel_to_brake_gain").as_double();
    command_timeout_ = get_parameter("command_timeout").as_double();
    stop_velocity_epsilon_ = get_parameter("stop_velocity_epsilon").as_double();

    command_pub_ = create_publisher<simulation::msg::VehicleControl>(
        "car_command", rclcpp::QoS(1));

    const auto trajectory_topic = get_parameter("trajectory_topic").as_string();
    const auto pose_topic = get_parameter("pose_topic").as_string();
    const auto twist_topic = get_parameter("twist_topic").as_string();

    trajectory_sub_ = create_subscription<planning::msg::Trajectory>(
        trajectory_topic,
        rclcpp::QoS(1),
        [this](const planning::msg::Trajectory::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_trajectory_ = msg;
          last_trajectory_time_ = now();
        });

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_pose_ = msg;
          last_pose_time_ = now();
        });

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        twist_topic,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_twist_ = msg;
          last_twist_time_ = now();
        });

    const double rate_hz = get_parameter("control_rate_hz").as_double();
    control_dt_ = 1.0 / std::max(1.0, rate_hz);
    timer_ = create_wall_timer(std::chrono::duration<double>(control_dt_),
                                std::bind(&TrajectoryFollowerNode::Tick, this));

    RCLCPP_INFO(
        get_logger(),
        "Following '%s', publishing VehicleControl on 'car_command'",
        trajectory_topic.c_str());
  }

private:
  struct TargetSelection {
    const planning::msg::TrajectoryPoint *point;
    double speed;
  };

  // Run one control cycle.
  void Tick() {
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
    geometry_msgs::msg::TwistStamped::ConstSharedPtr twist;
    planning::msg::Trajectory::ConstSharedPtr trajectory;
    rclcpp::Time last_trajectory_time;
    rclcpp::Time last_pose_time;
    rclcpp::Time last_twist_time;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pose = latest_pose_;
      twist = latest_twist_;
      trajectory = latest_trajectory_;
      last_trajectory_time = last_trajectory_time_;
      last_pose_time = last_pose_time_;
      last_twist_time = last_twist_time_;
    }

    const rclcpp::Time t = now();
    const bool trajectory_stale =
        !trajectory || trajectory->points.empty() ||
        (t - last_trajectory_time).seconds() > command_timeout_;
    const bool pose_stale =
        !pose || (t - last_pose_time).seconds() > command_timeout_;
    const bool twist_stale =
        !twist || (t - last_twist_time).seconds() > command_timeout_;
    if (trajectory_stale || pose_stale || twist_stale) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Control input timeout: trajectory=%s pose=%s twist=%s; full brake",
          trajectory_stale ? "stale" : "ok", pose_stale ? "stale" : "ok",
          twist_stale ? "stale" : "ok");
      PublishFailSafeStop();
      return;
    }

    const double current_speed =
        std::hypot(twist->twist.linear.x, twist->twist.linear.y);
    const TargetSelection target = FindLookaheadTarget(*trajectory, current_speed);

    if (target.speed <= stop_velocity_epsilon_) {
      // Treat a zero-speed trajectory as an explicit stop command.
      PublishFailSafeStop();
      return;
    }

    const double steering_cmd = SteeringToTarget(*pose, *target.point);
    const auto [throttle_cmd, brake_cmd] = ComputeLongitudinal(target.speed, current_speed);

    simulation::msg::VehicleControl cmd;
    cmd.throttle = static_cast<float>(throttle_cmd);
    cmd.steering = static_cast<float>(steering_cmd);
    cmd.brake = static_cast<float>(brake_cmd);
    cmd.reserved = 0.0F;
    command_pub_->publish(cmd);
  }

  // The same full-brake command is used for stale input and requested stops.
  void PublishFailSafeStop() {
    simulation::msg::VehicleControl cmd;
    cmd.throttle = 0.0F;
    cmd.steering = 0.0F;
    cmd.brake = 1.0F;
    cmd.reserved = 0.0F;
    command_pub_->publish(cmd);
    speed_integral_ = 0.0;
  }

  // Arc length prevents target jumps across spatially close path segments.
  TargetSelection FindLookaheadTarget(const planning::msg::Trajectory &trajectory,
                                      double current_speed) const {
    const double lookahead =
        std::clamp(
            lookahead_speed_gain_ * current_speed + lookahead_min_,
            lookahead_min_,
            lookahead_max_);

    size_t target_index = trajectory.points.size() - 1;
    double arc_length = 0.0;
    double target_speed = std::numeric_limits<double>::infinity();
    bool target_found = false;
    const bool has_positive_speed = std::any_of(
        trajectory.points.begin(), trajectory.points.end(), [this](const auto &point) {
          return point.velocity > stop_velocity_epsilon_;
        });
    const bool terminal_profile =
        has_positive_speed &&
        trajectory.points.back().velocity <= stop_velocity_epsilon_;
    const double preview_extra =
        terminal_profile ? 0.0 : speed_preview_extra_distance_;

    for (size_t i = 1; i < trajectory.points.size(); ++i) {
      const auto &previous = trajectory.points[i - 1].pose.position;
      const auto &current = trajectory.points[i].pose.position;
      arc_length += std::hypot(current.x - previous.x, current.y - previous.y);

      if (arc_length >= speed_preview_min_distance_) {
        target_speed = std::min(target_speed, static_cast<double>(trajectory.points[i].velocity));
      }
      if (!target_found && arc_length >= lookahead) {
        target_index = i;
        target_found = true;
      }
      if (target_found && arc_length >= lookahead + preview_extra) {
        break;
      }
    }

    if (!std::isfinite(target_speed)) {
      target_speed = trajectory.points[target_index].velocity;
    }

    return {&trajectory.points[target_index], target_speed};
  }

  double SteeringToTarget(const geometry_msgs::msg::PoseStamped &pose,
                           const planning::msg::TrajectoryPoint &target) const {
    const double ego_x = pose.pose.position.x;
    const double ego_y = pose.pose.position.y;
    const double yaw = control::QuaternionToYaw(pose.pose.orientation);

    const double dx = target.pose.position.x - ego_x;
    const double dy = target.pose.position.y - ego_y;
    const double local_x = dx * std::cos(yaw) + dy * std::sin(yaw);
    const double local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);
    const double target_dist = std::hypot(local_x, local_y);

    if (target_dist < 1e-3) {
      return 0.0;
    }

    const double alpha = std::atan2(local_y, local_x);

    // VehicleControl uses the opposite sign to the bicycle-model convention.
    const double delta = std::atan2(2.0 * wheelbase_ * std::sin(alpha), target_dist);
    return std::clamp(-delta / max_steering_angle_, -1.0, 1.0);
  }

  std::pair<double, double> ComputeLongitudinal(double target_speed, double current_speed) {
    const double error = target_speed - current_speed;
    speed_integral_ = std::clamp(
        speed_integral_ + error * control_dt_,
        -speed_integral_limit_,
        speed_integral_limit_);

    const double desired_accel = speed_kp_ * error + speed_ki_ * speed_integral_;

    if (desired_accel >= 0.0) {
      return {std::clamp(desired_accel * accel_to_throttle_gain_, 0.0, 1.0), 0.0};
    }

    return {0.0, std::clamp(-desired_accel * accel_to_brake_gain_, 0.0, 1.0)};
  }

  // Controller configuration.
  double wheelbase_{2.63};
  double max_steering_angle_{0.6};
  double lookahead_min_{2.0};
  double lookahead_max_{8.0};
  double lookahead_speed_gain_{0.6};
  double speed_preview_min_distance_{1.0};
  double speed_preview_extra_distance_{5.0};
  double speed_kp_{0.6};
  double speed_ki_{0.1};
  double speed_integral_limit_{2.0};
  double accel_to_throttle_gain_{1.0};
  double accel_to_brake_gain_{1.0};
  double control_dt_{0.05};
  double command_timeout_{0.5};
  double stop_velocity_epsilon_{0.05};
  double speed_integral_{0.0};

  // Latest inputs and their receive times.
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr latest_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr latest_twist_;
  planning::msg::Trajectory::ConstSharedPtr latest_trajectory_;
  rclcpp::Time last_trajectory_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_twist_time_{0, 0, RCL_ROS_TIME};

  // ROS interfaces.
  rclcpp::Subscription<planning::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<simulation::msg::VehicleControl>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
