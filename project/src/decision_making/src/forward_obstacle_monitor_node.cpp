// Projects the current depth cloud onto the planned path.
// Reports the nearest obstacle, its closing speed, and avoidance clearance.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <decision_making/msg/hazard_status.hpp>
#include <planning/msg/trajectory.hpp>

#include "math_utils.h"

class ForwardObstacleMonitorNode : public rclcpp::Node {
public:
  ForwardObstacleMonitorNode() : Node("forward_obstacle_monitor") {
    // Input topics and coordinate frame.
    declare_parameter<std::string>("points_topic", "/perception/points");
    declare_parameter<std::string>("world_frame", "world");
    declare_parameter<std::string>("pose_topic", "/OurCar/CoM/pose");
    declare_parameter<std::string>("trajectory_topic", "/planning/trajectory");

    // Main path corridor and height filters.
    declare_parameter<double>("lane_half_width", 1.75);
    declare_parameter<double>("max_range", 40.0);
    declare_parameter<double>("min_forward_distance", 0.5);
    declare_parameter<double>("trajectory_timeout", 1.0);
    declare_parameter<double>("ground_z", 0.0);
    declare_parameter<double>("min_obstacle_height", 0.5);
    declare_parameter<double>("max_obstacle_height", 1.8);
    declare_parameter<double>("edge_lateral_start", 1.3);
    declare_parameter<double>("edge_min_obstacle_height", 0.8);

    // Clearance check for a possible lateral avoidance path.
    declare_parameter<double>("avoid_shift", 2.2);
    declare_parameter<double>("avoid_check_half_width", 1.0);
    declare_parameter<double>("avoid_window_behind", 2.0);
    declare_parameter<double>("avoid_window_ahead", 2.0);
    declare_parameter<double>("avoid_check_min_height", 0.1);

    world_frame_ = get_parameter("world_frame").as_string();
    lane_half_width_ = get_parameter("lane_half_width").as_double();
    max_range_ = get_parameter("max_range").as_double();
    min_forward_distance_ = get_parameter("min_forward_distance").as_double();
    trajectory_timeout_ = get_parameter("trajectory_timeout").as_double();
    ground_z_ = get_parameter("ground_z").as_double();
    min_obstacle_height_ = get_parameter("min_obstacle_height").as_double();
    max_obstacle_height_ = get_parameter("max_obstacle_height").as_double();
    edge_lateral_start_ = get_parameter("edge_lateral_start").as_double();
    edge_min_obstacle_height_ = get_parameter("edge_min_obstacle_height").as_double();
    avoid_shift_ = get_parameter("avoid_shift").as_double();
    avoid_check_half_width_ = get_parameter("avoid_check_half_width").as_double();
    avoid_window_behind_ = get_parameter("avoid_window_behind").as_double();
    avoid_window_ahead_ = get_parameter("avoid_window_ahead").as_double();
    avoid_check_min_height_ = get_parameter("avoid_check_min_height").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    hazard_pub_ = create_publisher<decision_making::msg::HazardStatus>(
        "/decision_making/hazard_status", rclcpp::QoS(1));

    const auto pose_topic = get_parameter("pose_topic").as_string();
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic,
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) { latest_pose_ = msg; });

    const auto trajectory_topic = get_parameter("trajectory_topic").as_string();
    trajectory_sub_ = create_subscription<planning::msg::Trajectory>(
        trajectory_topic,
        rclcpp::QoS(1),
        [this](const planning::msg::Trajectory::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          latest_trajectory_ = msg;
          last_trajectory_time_ = now();
        });

    // Match the point cloud's sensor QoS.
    const auto points_topic = get_parameter("points_topic").as_string();
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic, rclcpp::SensorDataQoS(),
        std::bind(&ForwardObstacleMonitorNode::OnPoints, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Monitoring '%s' for obstacles within +/-%.2f m of the planned path, up to %.1f m ahead",
        points_topic.c_str(), lane_half_width_, max_range_);
  }

private:
  // Process one depth cloud and publish its nearest relevant path obstacle.
  void OnPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (!latest_pose_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      // Use the latest transform to tolerate small sensor/TF timestamp skew.
      transform = tf_buffer_->lookupTransform(
          world_frame_, msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Could not transform '%s' -> '%s': %s",
          msg->header.frame_id.c_str(), world_frame_.c_str(), ex.what());
      return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    const tf2::Matrix3x3 rotation(q);
    const tf2::Vector3 translation(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);

    const double ego_x = latest_pose_->pose.position.x;
    const double ego_y = latest_pose_->pose.position.y;

    planning::msg::Trajectory::ConstSharedPtr trajectory;
    rclcpp::Time last_trajectory_time;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      trajectory = latest_trajectory_;
      last_trajectory_time = last_trajectory_time_;
    }

    const bool trajectory_fresh =
        trajectory &&
        trajectory->points.size() >= 2 &&
        (now() - last_trajectory_time).seconds() < trajectory_timeout_;

    std::vector<PathVertex> path;
    if (trajectory_fresh) {
      path = BuildPathPolyline(*trajectory);
    }

    // Fall back to ego heading when no fresh path is available.
    const double yaw = decision_making::QuaternionToYaw(latest_pose_->pose.orientation);
    const double fwd_x = std::cos(yaw);
    const double fwd_y = std::sin(yaw);
    const double right_x = std::sin(yaw);
    const double right_y = -std::cos(yaw);

    double nearest = std::numeric_limits<double>::infinity();
    double nearest_lateral = 0.0;
    double nearest_height = 0.0;
    double nearest_x = 0.0;
    double nearest_y = 0.0;

    // Reuse filtered points for the later avoidance-clearance check.
    std::vector<std::pair<double, double>> candidates;  // (forward, lateral)

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const tf2::Vector3 world_pt = rotation * tf2::Vector3(*it_x, *it_y, *it_z) + translation;
      const double px = world_pt.x();
      const double py = world_pt.y();
      // Keep curbs for clearance checks, but reject near-zero ground points.
      const double height = world_pt.z() - ground_z_;
      if (height < avoid_check_min_height_ || height > max_obstacle_height_) {
        continue;
      }

      // Reject distant points before the more expensive path projection.
      const double candidate_range = std::hypot(px - ego_x, py - ego_y);
      const double clearance_range =
          max_range_ + lane_half_width_ + avoid_shift_ + avoid_check_half_width_;

      if (candidate_range > clearance_range) {
        continue;
      }

      double forward = 0.0;
      double lateral = 0.0;
      if (!path.empty()) {
        ProjectOntoPath(path, px, py, &forward, &lateral);
      } else {
        const double dx = px - ego_x;
        const double dy = py - ego_y;
        forward = dx * fwd_x + dy * fwd_y;
        lateral = dx * right_x + dy * right_y;
      }

      if (forward < min_forward_distance_ || forward > max_range_) {
        continue;
      }

      // Curbs also count when checking an avoidance lane.
      candidates.emplace_back(forward, lateral);

      // Main hazards require a greater height than clearance-only points.
      const double required_height =
          std::fabs(lateral) >= edge_lateral_start_
              ? edge_min_obstacle_height_
              : min_obstacle_height_;

      if (height >= required_height && std::fabs(lateral) <= lane_half_width_ &&
          forward < nearest) {
        nearest = forward;
        nearest_lateral = lateral;
        nearest_height = height;
        nearest_x = px;
        nearest_y = py;
      }
    }

    decision_making::msg::HazardStatus status;
    status.header.stamp = msg->header.stamp;
    status.header.frame_id = world_frame_;

    const bool detected = std::isfinite(nearest);
    status.detected = detected;
    status.distance = detected ? static_cast<float>(nearest) : static_cast<float>(max_range_);
    status.lateral_offset = detected ? static_cast<float>(nearest_lateral) : 0.0F;

    if (detected) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Nearest hazard: path_s=%.2f lateral=%.2f height=%.2f world=(%.2f, %.2f)",
          nearest, nearest_lateral, nearest_height, nearest_x, nearest_y);
    }

    // Check a short shifted corridor on the side opposite the obstacle.
    status.avoid_clear = false;
    if (detected) {
      const double shift_center = (nearest_lateral >= 0.0) ? -avoid_shift_ : avoid_shift_;
      const double window_lo = nearest - avoid_window_behind_;
      const double window_hi = nearest + avoid_window_ahead_;
      bool blocked = false;

      for (const auto &[c_forward, c_lateral] : candidates) {
        if (c_forward < window_lo || c_forward > window_hi) {
          continue;
        }
        if (std::fabs(c_lateral - shift_center) <= avoid_check_half_width_) {
          blocked = true;
          break;
        }
      }

      status.avoid_clear = !blocked;
    }

    const double stamp_sec = rclcpp::Time(msg->header.stamp).seconds();
    status.closing_speed = 0.0F;
    if (detected && have_last_ && last_detected_) {
      const double dt = stamp_sec - last_stamp_sec_;
      if (dt > 1e-3) {
        status.closing_speed = static_cast<float>((last_distance_ - nearest) / dt);
      }
    }

    last_distance_ = status.distance;
    last_detected_ = detected;
    last_stamp_sec_ = stamp_sec;
    have_last_ = true;

    hazard_pub_->publish(status);
  }

  struct PathVertex {
    double x;
    double y;
    double s;  // cumulative arc length from path[0] (~= the car's position when published)
  };

  // Convert the trajectory into a polyline with cumulative arc length.
  static std::vector<PathVertex> BuildPathPolyline(const planning::msg::Trajectory &trajectory) {
    std::vector<PathVertex> path;
    path.reserve(trajectory.points.size());
    double s = 0.0;

    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &p = trajectory.points[i].pose.position;
      if (i > 0) {
        s += std::hypot(p.x - path.back().x, p.y - path.back().y);
      }

      path.push_back({p.x, p.y, s});
    }
    return path;
  }

  // Return arc length and signed lateral distance to the closest path segment.
  static void ProjectOntoPath(
      const std::vector<PathVertex> &path,
      double px,
      double py,
      double *forward,
      double *lateral) {
    double best_dist_sq = std::numeric_limits<double>::infinity();
    double best_s = 0.0;
    double best_lateral = 0.0;

    for (size_t i = 0; i + 1 < path.size(); ++i) {
      const double segx = path[i + 1].x - path[i].x;
      const double segy = path[i + 1].y - path[i].y;
      const double seg_len_sq = segx * segx + segy * segy;
      if (seg_len_sq < 1e-9) {
        continue;
      }
      const double projection =
          ((px - path[i].x) * segx + (py - path[i].y) * segy) / seg_len_sq;
      const double t = std::clamp(projection, 0.0, 1.0);
      const double foot_x = path[i].x + t * segx;
      const double foot_y = path[i].y + t * segy;
      const double ddx = px - foot_x;
      const double ddy = py - foot_y;
      const double dist_sq = ddx * ddx + ddy * ddy;

      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_s = path[i].s + t * (path[i + 1].s - path[i].s);
        // Right of the path is positive.
        const double seg_len = std::sqrt(seg_len_sq);
        const double tangent_x = segx / seg_len;
        const double tangent_y = segy / seg_len;
        best_lateral = ddx * tangent_y - ddy * tangent_x;
      }
    }

    *forward = best_s;
    *lateral = best_lateral;
  }

  // Detection and avoidance configuration.
  std::string world_frame_{"world"};
  double lane_half_width_{1.75};
  double max_range_{40.0};
  double min_forward_distance_{0.5};
  double trajectory_timeout_{1.0};
  double ground_z_{0.0};
  double min_obstacle_height_{0.5};
  double max_obstacle_height_{1.8};
  double edge_lateral_start_{1.3};
  double edge_min_obstacle_height_{0.8};
  double avoid_shift_{2.2};
  double avoid_check_half_width_{1.0};
  double avoid_window_behind_{2.0};
  double avoid_window_ahead_{2.0};
  double avoid_check_min_height_{0.1};

  // Previous measurement used to estimate closing speed.
  bool have_last_{false};
  bool last_detected_{false};
  double last_distance_{0.0};
  double last_stamp_sec_{0.0};

  // Transform and input state.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr latest_pose_;

  std::mutex trajectory_mutex_;
  planning::msg::Trajectory::ConstSharedPtr latest_trajectory_;
  rclcpp::Time last_trajectory_time_{0, 0, RCL_ROS_TIME};

  // ROS interfaces.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<planning::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<decision_making::msg::HazardStatus>::SharedPtr hazard_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForwardObstacleMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
