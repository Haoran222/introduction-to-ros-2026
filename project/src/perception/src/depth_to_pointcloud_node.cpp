// Projects synchronized depth and CameraInfo messages into PointCloud2.
// The output remains in the depth camera optical frame.

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class DepthToPointCloudNode : public rclcpp::Node {
public:
  DepthToPointCloudNode() : Node("depth_to_pointcloud") {
    // Parameters describe the input stream and the density of the output cloud.
    declare_parameter<std::string>("depth_topic", "/OurCar/Sensors/DepthCamera/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/OurCar/Sensors/DepthCamera/camera_info");
    declare_parameter<int>("pixel_stride", 4);
    declare_parameter<double>("min_range", 0.3);
    declare_parameter<double>("max_range", 40.0);

    pixel_stride_ = std::max(1, static_cast<int>(get_parameter("pixel_stride").as_int()));
    min_range_ = get_parameter("min_range").as_double();
    max_range_ = get_parameter("max_range").as_double();

    // Camera frames use best-effort sensor QoS in the Unity bridge.
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());

    const auto depth_topic = get_parameter("depth_topic").as_string();
    const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
    const auto sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();

    image_sub_.subscribe(this, depth_topic, sensor_qos);
    info_sub_.subscribe(this, camera_info_topic, sensor_qos);

    // Image and CameraInfo timestamps match exactly.
    using DepthSynchronizer =
        message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

    sync_ = std::make_shared<DepthSynchronizer>(image_sub_, info_sub_, 10);
    sync_->registerCallback(
        std::bind(
            &DepthToPointCloudNode::OnDepth,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "Converting depth images from '%s' into point clouds", depth_topic.c_str());
  }

private:
  // Convert one synchronized depth/intrinsics pair into XYZ points.
  void OnDepth(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg) {
    if (depth_msg->encoding != "16UC1") {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Expected 16UC1 depth encoding, got '%s'",
          depth_msg->encoding.c_str());
      return;
    }

    const double fx = info_msg->k[0];
    const double fy = info_msg->k[4];
    const double cx = info_msg->k[2];
    const double cy = info_msg->k[5];

    if (fx <= 0.0 || fy <= 0.0) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Invalid camera intrinsics, dropping frame");
      return;
    }

    const auto *depth_row_base = reinterpret_cast<const uint16_t *>(depth_msg->data.data());
    const size_t row_stride_elems = depth_msg->step / sizeof(uint16_t);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = depth_msg->header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    const size_t max_points =
        (static_cast<size_t>(depth_msg->width) / pixel_stride_ + 1) *
        (static_cast<size_t>(depth_msg->height) / pixel_stride_ + 1);
    modifier.resize(max_points);

    sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

    size_t num_points = 0;
    for (uint32_t v = 0; v < depth_msg->height; v += pixel_stride_) {
      const uint16_t *depth_row = depth_row_base + v * row_stride_elems;
      for (uint32_t u = 0; u < depth_msg->width; u += pixel_stride_) {
        const uint16_t raw_mm = depth_row[u];

        if (raw_mm == 0) {
          continue;  // 0 encodes "no return" (see simulation's depth_camera_parser.h)
        }

        const double z = static_cast<double>(raw_mm) * 0.001;
        if (z < min_range_ || z > max_range_) {
          continue;
        }

        const double x = (static_cast<double>(u) - cx) * z / fx;
        const double y = (static_cast<double>(v) - cy) * z / fy;

        *out_x = static_cast<float>(x);
        *out_y = static_cast<float>(y);
        *out_z = static_cast<float>(z);
        ++out_x;
        ++out_y;
        ++out_z;
        ++num_points;
      }
    }

    modifier.resize(num_points);
    cloud.width = static_cast<uint32_t>(num_points);
    cloud.height = 1;
    cloud.is_dense = true;

    cloud_pub_->publish(cloud);
  }

  // Conversion settings.
  int pixel_stride_{4};
  double min_range_{0.3};
  double max_range_{40.0};

  // ROS interfaces.
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
  std::shared_ptr<
      message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>
      sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthToPointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
