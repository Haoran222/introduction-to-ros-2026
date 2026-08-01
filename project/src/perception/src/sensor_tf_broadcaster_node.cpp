// Publishes static transforms from the vehicle INS frame to all sensor frames.
// Frame names and offsets are configurable ROS parameters.

#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

namespace {

// Body frame to REP-103 camera optical frame rotation.
constexpr double kOpticalQx = 0.5;
constexpr double kOpticalQy = -0.5;
constexpr double kOpticalQz = 0.5;
constexpr double kOpticalQw = -0.5;

geometry_msgs::msg::TransformStamped MakeTransform(
    const rclcpp::Time &stamp, const std::string &parent, const std::string &child,
    double x, double y, double z, double qx, double qy, double qz, double qw) {
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = z;
  tf.transform.rotation.x = qx;
  tf.transform.rotation.y = qy;
  tf.transform.rotation.z = qz;
  tf.transform.rotation.w = qw;
  return tf;
}

}  // namespace

class SensorTfBroadcasterNode : public rclcpp::Node {
public:
  SensorTfBroadcasterNode() : Node("sensor_tf_broadcaster") {
    // Frame names.
    declare_parameter<std::string>("ins_frame", "OurCar/INS");
    declare_parameter<std::string>("center_frame", "OurCar/Center");
    declare_parameter<std::string>("sensor_base_frame", "OurCar/Sensors/SensorBase");
    declare_parameter<std::string>("seg_camera_frame", "OurCar/Sensors/SemanticCamera");
    declare_parameter<std::string>("depth_camera_frame", "OurCar/Sensors/DepthCamera");
    declare_parameter<std::string>("rgb_camera_left_frame", "OurCar/Sensors/RGBCameraLeft");
    declare_parameter<std::string>("rgb_camera_right_frame", "OurCar/Sensors/RGBCameraRight");
    declare_parameter<std::string>("imu_frame", "OurCar/Sensors/IMU");

    // Vehicle geometry from the assignment drawing.
    declare_parameter<double>("center_z", -0.7180116);

    declare_parameter<double>("sensor_base_x", 0.8199998);
    declare_parameter<double>("sensor_base_z", 1.269);

    declare_parameter<double>("rgb_camera_lateral_offset", 0.2);

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    BroadcastAll();
  }

private:
  // Build the static branch once; simulation publishes world -> INS.
  void BroadcastAll() {
    const auto ins_frame = get_parameter("ins_frame").as_string();
    const auto center_frame = get_parameter("center_frame").as_string();
    const auto sensor_base_frame = get_parameter("sensor_base_frame").as_string();
    const auto seg_camera_frame = get_parameter("seg_camera_frame").as_string();
    const auto depth_camera_frame = get_parameter("depth_camera_frame").as_string();
    const auto rgb_left_frame = get_parameter("rgb_camera_left_frame").as_string();
    const auto rgb_right_frame = get_parameter("rgb_camera_right_frame").as_string();
    const auto imu_frame = get_parameter("imu_frame").as_string();

    const double center_z = get_parameter("center_z").as_double();
    const double sensor_base_x = get_parameter("sensor_base_x").as_double();
    const double sensor_base_z = get_parameter("sensor_base_z").as_double();
    const double lateral_offset = get_parameter("rgb_camera_lateral_offset").as_double();

    const rclcpp::Time stamp = now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(7);

    transforms.push_back(MakeTransform(
        stamp, ins_frame, center_frame,
        0.0, 0.0, center_z,
        0.0, 0.0, 0.0, 1.0));

    transforms.push_back(MakeTransform(
        stamp, center_frame, sensor_base_frame,
        sensor_base_x, 0.0, sensor_base_z,
        0.0, 0.0, 0.0, 1.0));

    transforms.push_back(MakeTransform(
        stamp, sensor_base_frame, seg_camera_frame,
        0.0, 0.0, 0.0,
        kOpticalQx, kOpticalQy, kOpticalQz, kOpticalQw));

    transforms.push_back(MakeTransform(
        stamp, sensor_base_frame, depth_camera_frame,
        0.0, 0.0, 0.0,
        kOpticalQx, kOpticalQy, kOpticalQz, kOpticalQw));

    transforms.push_back(MakeTransform(
        stamp, sensor_base_frame, rgb_left_frame,
        0.0, lateral_offset, 0.0,
        kOpticalQx, kOpticalQy, kOpticalQz, kOpticalQw));

    transforms.push_back(MakeTransform(
        stamp, sensor_base_frame, rgb_right_frame,
        0.0, -lateral_offset, 0.0,
        kOpticalQx, kOpticalQy, kOpticalQz, kOpticalQw));

    // IMU and INS are co-located.
    transforms.push_back(MakeTransform(
        stamp, ins_frame, imu_frame,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0));

    broadcaster_->sendTransform(transforms);
    RCLCPP_INFO(
        get_logger(),
        "Broadcast %zu static sensor transforms rooted at '%s'",
        transforms.size(), ins_frame.c_str());
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorTfBroadcasterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
