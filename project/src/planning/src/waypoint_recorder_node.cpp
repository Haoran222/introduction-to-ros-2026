// Records a sparse route from the ego pose stream.
// Corners use a smaller spacing than straight road segments.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "math_utils.h"
#include "waypoint_io.h"

namespace {

// Wrap an angle into (-pi, pi].
double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

}  // namespace

class WaypointRecorderNode : public rclcpp::Node {
public:
  WaypointRecorderNode() : Node("waypoint_recorder") {
    // Recording thresholds keep straight segments sparse and corners dense.
    declare_parameter<std::string>("pose_topic", "/OurCar/CoM/pose");
    declare_parameter<std::string>("output_file", "/tmp/waypoints_recorded.yaml");
    declare_parameter<double>("min_spacing", 2.0);
    declare_parameter<double>("min_corner_spacing", 0.5);
    declare_parameter<double>("corner_yaw_threshold_deg", 12.0);

    output_file_ = get_parameter("output_file").as_string();
    min_spacing_ = get_parameter("min_spacing").as_double();
    min_corner_spacing_ = get_parameter("min_corner_spacing").as_double();
    corner_yaw_threshold_ = get_parameter("corner_yaw_threshold_deg").as_double() * M_PI / 180.0;

    const auto pose_topic = get_parameter("pose_topic").as_string();
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, rclcpp::QoS(10),
        std::bind(&WaypointRecorderNode::OnPose, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Recording waypoints from '%s' -> '%s': every %.1f m on straights, "
                "densified to every %.1f m once heading changes by > %.0f deg. "
                "Drive the track once, then Ctrl+C when you reach the goal.",
                pose_topic.c_str(), output_file_.c_str(), min_spacing_, min_corner_spacing_,
                get_parameter("corner_yaw_threshold_deg").as_double());
  }

private:
  // Save immediately after accepting a sample so Ctrl+C cannot lose the route.
  void OnPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
    const double x = msg->pose.position.x;
    const double y = msg->pose.position.y;
    const double yaw = planning::QuaternionToYaw(msg->pose.orientation);

    if (!waypoints_.empty()) {
      const double dist = std::hypot(x - waypoints_.back().x, y - waypoints_.back().y);
      const double yaw_change = std::fabs(NormalizeAngle(yaw - waypoints_.back().yaw));
      const bool turning_enough =
          yaw_change >= corner_yaw_threshold_ && dist >= min_corner_spacing_;

      if (dist < min_spacing_ && !turning_enough) {
        return;
      }
    }

    waypoints_.push_back({x, y, yaw});
    try {
      planning::SaveWaypoints(output_file_, waypoints_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to save waypoints: %s", e.what());
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Recorded waypoint #%zu at (%.2f, %.2f, yaw=%.2f)",
        waypoints_.size(), x, y, yaw);
  }

  // Configuration.
  std::string output_file_;
  double min_spacing_{2.0};
  double min_corner_spacing_{0.5};
  double corner_yaw_threshold_{12.0 * M_PI / 180.0};

  // Recorded data and ROS input.
  std::vector<planning::Waypoint> waypoints_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointRecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
