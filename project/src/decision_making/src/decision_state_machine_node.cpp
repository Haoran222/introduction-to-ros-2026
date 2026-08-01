// Gates the planned trajectory using traffic-light and hazard inputs.
// Hysteresis prevents rapid state changes caused by noisy perception.

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <decision_making/msg/hazard_status.hpp>
#include <decision_making/srv/set_autonomy_enabled.hpp>
#include <planning/msg/trajectory.hpp>

namespace {

enum class State {
  kAutonomyDisabled,
  kDrive,
  kTrafficStop,
  kCaution,
  kEmergencyStop,
  kAvoiding,
  kSensorFault,
};

const char *StateName(State s) {
  switch (s) {
    case State::kAutonomyDisabled:
      return "AUTONOMY_DISABLED";
    case State::kDrive:
      return "DRIVE";
    case State::kTrafficStop:
      return "TRAFFIC_STOP";
    case State::kCaution:
      return "CAUTION";
    case State::kEmergencyStop:
      return "EMERGENCY_STOP";
    case State::kAvoiding:
      return "AVOIDING";
    case State::kSensorFault:
      return "SENSOR_FAULT";
  }
  return "UNKNOWN";
}

}  // namespace

class DecisionStateMachineNode : public rclcpp::Node {
public:
  DecisionStateMachineNode() : Node("decision_state_machine") {
    // Input topics.
    declare_parameter<std::string>("must_stop_topic", "/traffic_light/must_stop");
    declare_parameter<std::string>("hazard_topic", "/decision_making/hazard_status");
    declare_parameter<std::string>("trajectory_topic", "/planning/trajectory");

    // Hazard thresholds and state timing.
    declare_parameter<double>("caution_distance", 15.0);
    declare_parameter<double>("caution_min_speed_factor", 0.3);
    declare_parameter<double>("emergency_distance", 6.0);
    declare_parameter<double>("emergency_ttc", 2.0);
    declare_parameter<double>("decision_rate_hz", 10.0);
    declare_parameter<double>("state_hold_time", 0.3);
    declare_parameter<double>("emergency_min_dwell", 1.0);

    // Lateral avoidance behavior.
    declare_parameter<double>("avoid_after_time", 3.0);
    declare_parameter<double>("avoid_max_closing_speed", 0.3);
    declare_parameter<double>("avoid_abort_closing_speed", 1.5);
    declare_parameter<double>("avoid_abort_closing_hold", 0.5);
    declare_parameter<double>("avoid_shift", 2.2);
    declare_parameter<double>("avoid_ramp_in", 3.0);
    declare_parameter<double>("avoid_hold_distance", 4.0);
    declare_parameter<double>("avoid_ramp_out", 3.0);
    declare_parameter<double>("avoid_speed_factor", 0.4);
    declare_parameter<double>("avoid_clear_loss_hold", 0.7);

    // Input freshness watchdogs.
    declare_parameter<double>("hazard_timeout", 1.0);
    declare_parameter<double>("trajectory_timeout", 1.0);

    caution_distance_ = get_parameter("caution_distance").as_double();
    caution_min_speed_factor_ = get_parameter("caution_min_speed_factor").as_double();
    emergency_distance_ = get_parameter("emergency_distance").as_double();
    emergency_ttc_ = get_parameter("emergency_ttc").as_double();
    state_hold_time_ = get_parameter("state_hold_time").as_double();
    emergency_min_dwell_ = get_parameter("emergency_min_dwell").as_double();
    avoid_after_time_ = get_parameter("avoid_after_time").as_double();
    avoid_max_closing_speed_ = get_parameter("avoid_max_closing_speed").as_double();
    avoid_abort_closing_speed_ = get_parameter("avoid_abort_closing_speed").as_double();
    avoid_abort_closing_hold_ = get_parameter("avoid_abort_closing_hold").as_double();
    avoid_shift_ = get_parameter("avoid_shift").as_double();
    avoid_ramp_in_ = get_parameter("avoid_ramp_in").as_double();
    avoid_hold_distance_ = get_parameter("avoid_hold_distance").as_double();
    avoid_ramp_out_ = get_parameter("avoid_ramp_out").as_double();
    avoid_speed_factor_ = get_parameter("avoid_speed_factor").as_double();
    avoid_clear_loss_hold_ = get_parameter("avoid_clear_loss_hold").as_double();
    hazard_timeout_ = get_parameter("hazard_timeout").as_double();
    trajectory_timeout_ = get_parameter("trajectory_timeout").as_double();

    trajectory_pub_ = create_publisher<planning::msg::Trajectory>(
        "/decision_making/trajectory", rclcpp::QoS(1));
    state_pub_ = create_publisher<std_msgs::msg::String>(
        "/decision_making/state", rclcpp::QoS(1));

    const auto must_stop_topic = get_parameter("must_stop_topic").as_string();
    const auto hazard_topic = get_parameter("hazard_topic").as_string();
    const auto trajectory_topic = get_parameter("trajectory_topic").as_string();

    must_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        must_stop_topic,
        rclcpp::QoS(1),
        [this](const std_msgs::msg::Bool::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          must_stop_ = msg->data;
        });

    hazard_sub_ = create_subscription<decision_making::msg::HazardStatus>(
        hazard_topic,
        rclcpp::QoS(1),
        [this](const decision_making::msg::HazardStatus::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_hazard_ = msg;
          last_hazard_time_ = now();
        });

    trajectory_sub_ = create_subscription<planning::msg::Trajectory>(
        trajectory_topic,
        rclcpp::QoS(1),
        [this](const planning::msg::Trajectory::ConstSharedPtr &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_trajectory_ = msg;
          last_trajectory_time_ = now();
        });

    autonomy_service_ = create_service<decision_making::srv::SetAutonomyEnabled>(
        "/decision_making/set_autonomy_enabled",
        std::bind(
            &DecisionStateMachineNode::SetAutonomyEnabled,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    const double rate_hz = get_parameter("decision_rate_hz").as_double();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz)),
                                std::bind(&DecisionStateMachineNode::Tick, this));
  }

private:
  // Evaluate inputs, commit one state, and publish a gated trajectory.
  void Tick() {
    bool autonomy_enabled;
    bool must_stop;
    decision_making::msg::HazardStatus::ConstSharedPtr hazard;
    planning::msg::Trajectory::ConstSharedPtr trajectory;
    rclcpp::Time last_hazard_time;
    rclcpp::Time last_trajectory_time;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      autonomy_enabled = autonomy_enabled_;
      must_stop = must_stop_;
      hazard = latest_hazard_;
      trajectory = latest_trajectory_;
      last_hazard_time = last_hazard_time_;
      last_trajectory_time = last_trajectory_time_;
    }

    const rclcpp::Time t = now();
    const bool hazard_stale =
        !hazard || (t - last_hazard_time).seconds() > hazard_timeout_;
    const bool trajectory_stale =
        !trajectory || trajectory->points.empty() ||
        (t - last_trajectory_time).seconds() > trajectory_timeout_;
    const bool sensor_fault = hazard_stale || trajectory_stale;
    if (sensor_fault) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Input timeout: hazard_stale=%s trajectory_stale=%s; holding a safe stop",
          hazard_stale ? "true" : "false", trajectory_stale ? "true" : "false");
      hazard.reset();
    }

    UpdateHazardContinuity(hazard);

    const State candidate =
        !autonomy_enabled
            ? State::kAutonomyDisabled
            : (sensor_fault ? State::kSensorFault : DecideState(must_stop, hazard));
    const State previous_state = committed_state_;
    const State state = ApplyHysteresis(candidate);

    if (state == State::kAvoiding && previous_state != State::kAvoiding && hazard) {
      // Keep one avoidance side for the complete manoeuvre.
      active_avoid_shift_sign_ = (hazard->lateral_offset >= 0.0F) ? -1.0 : 1.0;
    }
    if (state != State::kAvoiding) {
      avoid_clear_lost_ = false;
    }
    if (state != last_logged_state_) {
      RCLCPP_INFO(
          get_logger(),
          "State: %s -> %s",
          StateName(last_logged_state_), StateName(state));
      last_logged_state_ = state;
    }

    std_msgs::msg::String state_msg;
    state_msg.data = StateName(state);
    state_pub_->publish(state_msg);

    if (trajectory_stale) {
      planning::msg::Trajectory empty;
      empty.header.stamp = t;
      empty.header.frame_id = "world";
      trajectory_pub_->publish(empty);
      return;
    }

    const double factor = SpeedFactor(state, hazard);
    planning::msg::Trajectory gated = *trajectory;

    for (auto &point : gated.points) {
      point.velocity = static_cast<float>(std::max(0.0, point.velocity * factor));
    }
    if (state == State::kAvoiding && hazard) {
      ApplyAvoidanceShift(gated, *hazard);
    }

    trajectory_pub_->publish(gated);
  }

  // Handle an operator request to stop or resume autonomous driving. Disabling
  // takes effect on the next state-machine tick and is treated as an immediate,
  // fail-safe transition by ApplyHysteresis().
  void SetAutonomyEnabled(
      const std::shared_ptr<decision_making::srv::SetAutonomyEnabled::Request> request,
      std::shared_ptr<decision_making::srv::SetAutonomyEnabled::Response> response) {
    bool changed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      changed = autonomy_enabled_ != request->enable;
      autonomy_enabled_ = request->enable;
      response->enabled = autonomy_enabled_;
    }

    response->success = true;
    response->message = changed
                            ? (response->enabled ? "Autonomous driving enabled"
                                                 : "Autonomous driving disabled; safe stop requested")
                            : (response->enabled ? "Autonomous driving was already enabled"
                                                 : "Autonomous driving was already disabled");
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  // Track continuous detection before treating an obstacle as stationary.
  void UpdateHazardContinuity(const decision_making::msg::HazardStatus::ConstSharedPtr &hazard) {
    const bool detected = hazard && hazard->detected;
    if (detected && !hazard_active_) {
      hazard_continuous_since_ = now();
    }
    hazard_active_ = detected;
  }

  bool CanAvoid(const decision_making::msg::HazardStatus::ConstSharedPtr &hazard) const {
    if (!hazard || !hazard->detected || !hazard->avoid_clear) {
      return false;
    }
    // Avoid only obstacles with a low closing speed.
    if (hazard->closing_speed > avoid_max_closing_speed_) {
      return false;
    }
    if (!hazard_active_ || (now() - hazard_continuous_since_).seconds() < avoid_after_time_) {
      return false;
    }
    return true;
  }

  // Apply a smooth ramp-in, hold, and ramp-out lateral path shift.
  void ApplyAvoidanceShift(planning::msg::Trajectory &trajectory,
                            const decision_making::msg::HazardStatus &hazard) const {
    auto &points = trajectory.points;
    if (points.size() < 2) {
      return;
    }
    const double obstacle_s = hazard.distance;
    // Clamp the ramp to the current path origin.
    const double ramp_start = std::max(0.0, obstacle_s - avoid_ramp_in_);
    const double ramp_in_len = std::max(obstacle_s - ramp_start, 1e-3);

    // Measure arc length and tangents from an unchanged path snapshot.
    const auto original = points;

    double s = 0.0;
    for (size_t i = 0; i < original.size(); ++i) {
      if (i > 0) {
        const double dx = original[i].pose.position.x - original[i - 1].pose.position.x;
        const double dy = original[i].pose.position.y - original[i - 1].pose.position.y;
        s += std::hypot(dx, dy);
      }

      double ramp = 0.0;
      if (s >= ramp_start && s < obstacle_s) {
        ramp = (s - ramp_start) / ramp_in_len;
      } else if (s >= obstacle_s && s <= obstacle_s + avoid_hold_distance_) {
        ramp = 1.0;
      } else if (s > obstacle_s + avoid_hold_distance_ && s <= obstacle_s + avoid_hold_distance_ + avoid_ramp_out_) {
        ramp = 1.0 - (s - (obstacle_s + avoid_hold_distance_)) / std::max(avoid_ramp_out_, 1e-3);
      }
      if (ramp <= 0.0) {
        continue;
      }

      // Estimate the local path tangent from neighboring points.
      double tangent_x = 1.0;
      double tangent_y = 0.0;
      if (original.size() >= 2) {
        const auto &prev = original[(i == 0) ? 0 : i - 1].pose.position;
        const auto &next = original[(i + 1 < original.size()) ? i + 1 : i].pose.position;
        const double tx = next.x - prev.x;
        const double ty = next.y - prev.y;
        const double tlen = std::hypot(tx, ty);
        if (tlen > 1e-6) {
          tangent_x = tx / tlen;
          tangent_y = ty / tlen;
        }
      }
      // Right of the path is positive.
      const double right_x = tangent_y;
      const double right_y = -tangent_x;
      const double offset = active_avoid_shift_sign_ * avoid_shift_ * ramp;
      points[i].pose.position.x = original[i].pose.position.x + right_x * offset;
      points[i].pose.position.y = original[i].pose.position.y + right_y * offset;
    }
  }

  // Delay normal transitions, but enter disabled, emergency, and fault states now.
  // Keep emergency braking active for a minimum dwell time.
  State ApplyHysteresis(State candidate) {
    const rclcpp::Time t = now();

    if (candidate == State::kAutonomyDisabled ||
        candidate == State::kEmergencyStop ||
        candidate == State::kSensorFault) {
      if (candidate == State::kEmergencyStop &&
          committed_state_ != State::kEmergencyStop) {
        emergency_entered_at_ = t;
      }
      committed_state_ = candidate;
      pending_state_ = candidate;
      pending_since_ = t;
      return committed_state_;
    }

    if (committed_state_ == State::kEmergencyStop &&
        (t - emergency_entered_at_).seconds() < emergency_min_dwell_) {
      return committed_state_;
    }

    if (candidate != pending_state_) {
      pending_state_ = candidate;
      pending_since_ = t;
    }
    if ((t - pending_since_).seconds() >= state_hold_time_) {
      committed_state_ = pending_state_;
    }
    return committed_state_;
  }

  bool ContinueCommittedAvoidance(
      const decision_making::msg::HazardStatus::ConstSharedPtr &hazard) {
    if (committed_state_ != State::kAvoiding || !hazard || !hazard->detected) {
      avoid_clear_lost_ = false;
      avoid_closing_high_ = false;
      return false;
    }
    // Ignore one-frame closing-speed spikes during committed avoidance.
    if (hazard->closing_speed > avoid_abort_closing_speed_) {
      if (!avoid_closing_high_) {
        avoid_closing_high_ = true;
        avoid_closing_high_since_ = now();
      }
      if ((now() - avoid_closing_high_since_).seconds() >= avoid_abort_closing_hold_) {
        avoid_clear_lost_ = false;
        return false;
      }
    } else {
      avoid_closing_high_ = false;
    }
    if (hazard->avoid_clear) {
      avoid_clear_lost_ = false;
      return true;
    }
    if (!avoid_clear_lost_) {
      avoid_clear_lost_ = true;
      avoid_clear_lost_since_ = now();
    }
    return (now() - avoid_clear_lost_since_).seconds() < avoid_clear_loss_hold_;
  }

  State DecideState(
      bool must_stop,
      const decision_making::msg::HazardStatus::ConstSharedPtr &hazard) {
    if (hazard && hazard->detected) {
      const bool too_close = hazard->distance < emergency_distance_;
      const bool closing_too_fast =
          hazard->closing_speed > 0.0F &&
          (hazard->distance / std::max(hazard->closing_speed, 0.1F)) < emergency_ttc_;

      if (too_close || closing_too_fast) {
        return (ContinueCommittedAvoidance(hazard) || CanAvoid(hazard))
                   ? State::kAvoiding
                   : State::kEmergencyStop;
      }
    }

    if (must_stop) {
      return State::kTrafficStop;
    }

    if (hazard && hazard->detected && hazard->distance < caution_distance_) {
      return (ContinueCommittedAvoidance(hazard) || CanAvoid(hazard))
                 ? State::kAvoiding
                 : State::kCaution;
    }

    return State::kDrive;
  }

  double SpeedFactor(
      State state,
      const decision_making::msg::HazardStatus::ConstSharedPtr &hazard) const {
    switch (state) {
      case State::kAutonomyDisabled:
        return 0.0;
      case State::kDrive:
        return 1.0;
      case State::kTrafficStop:
      case State::kEmergencyStop:
      case State::kSensorFault:
        return 0.0;
      case State::kAvoiding:
        return avoid_speed_factor_;
      case State::kCaution: {
        const double ratio = hazard ? (hazard->distance / caution_distance_) : 1.0;
        return std::clamp(ratio, caution_min_speed_factor_, 1.0);
      }
    }

    return 1.0;
  }

  // Decision thresholds and avoidance configuration.
  double caution_distance_{15.0};
  double caution_min_speed_factor_{0.3};
  double emergency_distance_{6.0};
  double emergency_ttc_{2.0};
  double state_hold_time_{0.3};
  double emergency_min_dwell_{1.0};
  double avoid_after_time_{3.0};
  double avoid_max_closing_speed_{0.3};
  double avoid_abort_closing_speed_{1.5};
  double avoid_abort_closing_hold_{0.5};
  double avoid_shift_{2.2};
  double avoid_ramp_in_{3.0};
  double avoid_hold_distance_{4.0};
  double avoid_ramp_out_{3.0};
  double avoid_speed_factor_{0.4};
  double avoid_clear_loss_hold_{0.7};
  double hazard_timeout_{1.0};
  double trajectory_timeout_{1.0};
  double active_avoid_shift_sign_{1.0};
  State last_logged_state_{State::kDrive};

  // State-machine memory used by hysteresis and avoidance continuity.
  State committed_state_{State::kDrive};
  State pending_state_{State::kDrive};
  rclcpp::Time pending_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time emergency_entered_at_{0, 0, RCL_ROS_TIME};
  bool hazard_active_{false};
  rclcpp::Time hazard_continuous_since_{0, 0, RCL_ROS_TIME};
  bool avoid_clear_lost_{false};
  rclcpp::Time avoid_clear_lost_since_{0, 0, RCL_ROS_TIME};
  bool avoid_closing_high_{false};
  rclcpp::Time avoid_closing_high_since_{0, 0, RCL_ROS_TIME};

  // Latest inputs and receive times.
  std::mutex mutex_;
  bool autonomy_enabled_{true};
  bool must_stop_{false};
  decision_making::msg::HazardStatus::ConstSharedPtr latest_hazard_;
  planning::msg::Trajectory::ConstSharedPtr latest_trajectory_;
  rclcpp::Time last_hazard_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_trajectory_time_{0, 0, RCL_ROS_TIME};

  // ROS interfaces.
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr must_stop_sub_;
  rclcpp::Subscription<decision_making::msg::HazardStatus>::SharedPtr hazard_sub_;
  rclcpp::Subscription<planning::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<planning::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<decision_making::srv::SetAutonomyEnabled>::SharedPtr autonomy_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DecisionStateMachineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
