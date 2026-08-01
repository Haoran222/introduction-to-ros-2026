// Tracks progress along the recorded route.
// Publishes the next goal and a local waypoint window.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "math_utils.h"
#include "waypoint_io.h"

class GoalSelectorNode : public rclcpp::Node {
public:
  GoalSelectorNode() : Node("goal_selector") {
    // Route source and ego pose input.
    declare_parameter<std::string>("waypoints_file", "");
    declare_parameter<std::string>("pose_topic", "/OurCar/CoM/pose");

    // Route progress and lookahead settings.
    declare_parameter<double>("arrival_radius", 1.5);
    declare_parameter<double>("final_arrival_radius", 3.0);
    declare_parameter<int>("lookahead_count", 15);
    declare_parameter<int>("progress_search_count", 30);
    declare_parameter<double>("initial_heading_weight", 2.0);

    arrival_radius_ = get_parameter("arrival_radius").as_double();
    final_arrival_radius_ = get_parameter("final_arrival_radius").as_double();
    lookahead_count_ = std::max(2, static_cast<int>(get_parameter("lookahead_count").as_int()));
    progress_search_count_ =
        std::max(2, static_cast<int>(get_parameter("progress_search_count").as_int()));
    initial_heading_weight_ = get_parameter("initial_heading_weight").as_double();

    const auto waypoints_file = get_parameter("waypoints_file").as_string();
    if (waypoints_file.empty()) {
      RCLCPP_ERROR(get_logger(), "'waypoints_file' parameter is required (see config/waypoints.yaml).");
    } else {
      try {
        waypoints_ = planning::LoadWaypoints(waypoints_file);
        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from '%s'", waypoints_.size(), waypoints_file.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Failed to load waypoints from '%s': %s", waypoints_file.c_str(), e.what());
      }
    }

    if (waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(),
                    "No waypoints loaded -- run waypoint_recorder_node once to record the track, then point "
                    "'waypoints_file' at the result.");
    }

    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/planning/next_goal", rclcpp::QoS(1));
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/planning/local_path", rclcpp::QoS(1));
    goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/planning/goal_reached", rclcpp::QoS(1));

    const auto pose_topic = get_parameter("pose_topic").as_string();
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic,
        rclcpp::QoS(10),
        std::bind(&GoalSelectorNode::OnPose, this, std::placeholders::_1));
  }

private:
  // Update monotonic route progress for every ego pose.
  void OnPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
    if (waypoints_.empty()) {
      return;
    }

    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;
    const double yaw = planning::QuaternionToYaw(msg->pose.orientation);

    if (!progress_initialized_) {
      InitializeProgress(x, y, yaw);
      progress_initialized_ = true;
    }

    RecoverForwardProgress(x, y);

    // Advance past all waypoints reached by this pose update.
    while (
        current_index_ + 1 < waypoints_.size() &&
        Distance(x, y, waypoints_[current_index_]) < arrival_radius_) {
      ++current_index_;
    }

    const bool at_final_goal =
        (current_index_ + 1 == waypoints_.size()) &&
        Distance(x, y, waypoints_[current_index_]) < final_arrival_radius_;
    if (at_final_goal && !goal_reached_latched_) {
      goal_reached_latched_ = true;
      RCLCPP_INFO(get_logger(), "Final goal reached.");
    }

    std_msgs::msg::Bool reached_msg;
    reached_msg.data = goal_reached_latched_;
    goal_reached_pub_->publish(reached_msg);

    PublishNextGoal(msg->header.stamp);
    PublishLocalPath(msg->header.stamp);
  }

  // Geometry helpers are stateless and kept together for easy unit testing.
  static double Distance(double x, double y, const planning::Waypoint &wp) {
    return std::hypot(x - wp.x, y - wp.y);
  }

  static double NormalizeAngle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double DistanceToSegment(double x, double y, const planning::Waypoint &a,
                                  const planning::Waypoint &b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    if (length_sq < 1e-9) {
      return Distance(x, y, a);
    }
    const double projection =
        std::clamp(((x - a.x) * dx + (y - a.y) * dy) / length_sq, 0.0, 1.0);
    return std::hypot(x - (a.x + projection * dx), y - (a.y + projection * dy));
  }

  // Initialize on the nearest segment, using heading as a tie-breaker.
  void InitializeProgress(double x, double y, double yaw) {
    if (waypoints_.size() < 2) {
      current_index_ = 0;
      return;
    }

    size_t best_segment = 0;
    double best_score = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < waypoints_.size(); ++i) {
      const auto &a = waypoints_[i];
      const auto &b = waypoints_[i + 1];
      const double segment_yaw = std::atan2(b.y - a.y, b.x - a.x);
      const double heading_error = std::fabs(NormalizeAngle(yaw - segment_yaw));
      const double score =
          DistanceToSegment(x, y, a, b) + initial_heading_weight_ * heading_error;
      if (score < best_score) {
        best_score = score;
        best_segment = i;
      }
    }

    current_index_ = std::min(best_segment + 1, waypoints_.size() - 1);
    RCLCPP_INFO(get_logger(), "Initialized route progress at waypoint %zu/%zu",
                current_index_, waypoints_.size() - 1);
  }

  // Recover missed progress within a bounded forward-only search window.
  void RecoverForwardProgress(double x, double y) {
    if (waypoints_.size() < 2 || current_index_ + 1 >= waypoints_.size()) {
      return;
    }

    const size_t search_start = current_index_ > 0 ? current_index_ - 1 : 0;
    const size_t last_segment = waypoints_.size() - 2;
    const size_t search_end =
        std::min(last_segment, current_index_ + static_cast<size_t>(progress_search_count_));

    size_t best_segment = search_start;
    double best_distance = std::numeric_limits<double>::infinity();
    for (size_t i = search_start; i <= search_end; ++i) {
      const double distance =
          DistanceToSegment(x, y, waypoints_[i], waypoints_[i + 1]);
      if (distance < best_distance) {
        best_distance = distance;
        best_segment = i;
      }
    }

    const size_t recovered_index =
        std::max(current_index_, std::min(best_segment + 1, waypoints_.size() - 1));
    if (recovered_index > current_index_) {
      const size_t skipped = recovered_index - current_index_;
      current_index_ = recovered_index;
      if (skipped > 1) {
        RCLCPP_WARN(get_logger(),
                    "Recovered forward route progress: skipped %zu missed waypoints, now %zu/%zu",
                    skipped, current_index_, waypoints_.size() - 1);
      }
    }
  }

  void PublishNextGoal(const rclcpp::Time &stamp) {
    const auto &wp = waypoints_[current_index_];

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = stamp;
    goal.header.frame_id = "world";
    goal.pose.position.x = wp.x;
    goal.pose.position.y = wp.y;
    goal.pose.orientation = planning::YawToQuaternion(wp.yaw);
    goal_pub_->publish(goal);
  }

  void PublishLocalPath(const rclcpp::Time &stamp) {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = "world";

    const size_t end = std::min(
        waypoints_.size(),
        current_index_ + static_cast<size_t>(lookahead_count_));

    for (size_t i = current_index_; i < end; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = waypoints_[i].x;
      pose.pose.position.y = waypoints_[i].y;
      pose.pose.orientation = planning::YawToQuaternion(waypoints_[i].yaw);
      path.poses.push_back(pose);
    }

    local_path_pub_->publish(path);
  }

  // Route data and progress state.
  std::vector<planning::Waypoint> waypoints_;
  size_t current_index_{0};
  double arrival_radius_{1.5};
  double final_arrival_radius_{3.0};
  int lookahead_count_{8};
  int progress_search_count_{30};
  double initial_heading_weight_{2.0};
  bool progress_initialized_{false};
  bool goal_reached_latched_{false};

  // ROS interfaces.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalSelectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
