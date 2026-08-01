#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <simulation/msg/vehicle_control.hpp>

namespace {

constexpr float kLoopInterval = 0.05F;
constexpr float kThrottle = 0.5F;
constexpr float kSteeringAmplitude = 0.5F;
constexpr float kTwoPi = 6.28F;

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("dummy_controller_node");
  auto command_pub = node->create_publisher<simulation::msg::VehicleControl>(
      "car_command", rclcpp::QoS(1));

  rclcpp::Rate loop_rate(1.0F / kLoopInterval);
  float elapsed_time = 0.0F;

  while (rclcpp::ok()) {
    simulation::msg::VehicleControl msg;
    msg.throttle = kThrottle;
    msg.steering = std::sin(kTwoPi * elapsed_time) * kSteeringAmplitude;
    msg.brake = 0.0F;
    msg.reserved = 0.0F;

    command_pub->publish(msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
    elapsed_time += kLoopInterval;
  }

  rclcpp::shutdown();
  return 0;
}
