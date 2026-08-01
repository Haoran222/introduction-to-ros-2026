// Detects traffic lights from the RGB image using HSV and shape filters.
// Results are debounced before publishing the state and stop request.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include "perception/msg/traffic_light_state.hpp"

namespace {

struct ColorRange {
  cv::Scalar low_a;
  cv::Scalar high_a;
  cv::Scalar low_b;
  cv::Scalar high_b;
  bool has_second_band;
};

struct Candidate {
  double area{0.0};
  double circularity{0.0};
  cv::Rect bounds;
  uint8_t state{0};
};

double Circularity(const std::vector<cv::Point> &contour, double area) {
  const double perimeter = cv::arcLength(contour, true);
  if (perimeter <= 1e-6) {
    return 0.0;
  }

  // 1.0 for a perfect circle, lower for elongated/irregular shapes.
  return 4.0 * M_PI * area / (perimeter * perimeter);
}

double ColorFillFraction(const cv::Mat &color_mask, const cv::Rect &bounds) {
  return static_cast<double>(cv::countNonZero(color_mask(bounds))) /
         static_cast<double>(std::max(1, bounds.area()));
}

// Measure the yellow housing around a candidate lamp.
double YellowHousingSurroundFraction(const cv::Mat &hsv, const cv::Rect &bounds) {
  const int margin_x = std::max(4, bounds.width);
  const int margin_y = std::max(4, bounds.height);
  const cv::Rect expanded =
      cv::Rect(bounds.x - margin_x, bounds.y - margin_y, bounds.width + 2 * margin_x, bounds.height + 2 * margin_y) &
      cv::Rect(0, 0, hsv.cols, hsv.rows);

  long housing_count = 0;
  long total = 0;
  for (int y = expanded.y; y < expanded.y + expanded.height; ++y) {
    const auto *hsv_row = hsv.ptr<cv::Vec3b>(y);
    for (int x = expanded.x; x < expanded.x + expanded.width; ++x) {
      if (bounds.contains(cv::Point(x, y))) {
        continue;
      }
      ++total;
      const auto &pixel = hsv_row[x];
      if (pixel[0] >= 15 && pixel[0] <= 40 && pixel[1] >= 80 && pixel[2] >= 100) {
        ++housing_count;
      }
    }
  }
  return total > 0 ? static_cast<double>(housing_count) / static_cast<double>(total) : 0.0;
}

// Verify the simulator's vertical three-lamp housing.
bool HasAlignedHousingStack(const cv::Mat &housing_mask, const cv::Rect &bounds) {
  const int center_x = bounds.x + bounds.width / 2;
  const int center_y = bounds.y + bounds.height / 2;
  const int half_width = std::max(6, 2 * bounds.width);
  const int radius_y = std::max(30, 10 * bounds.height);
  const cv::Rect search =
      cv::Rect(
          center_x - half_width,
          center_y - radius_y,
          2 * half_width + 1,
          2 * radius_y + 1) &
      cv::Rect(0, 0, housing_mask.cols, housing_mask.rows);

  cv::Mat local_mask = housing_mask(search).clone();
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(local_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  int lamp_component_count = 0;
  for (const auto &contour : contours) {
    const cv::Rect component = cv::boundingRect(contour);

    if (cv::contourArea(contour) >= 2.0 && component.width >= 2 && component.height >= 2) {
      ++lamp_component_count;
    }
  }

  return lamp_component_count >= 2;
}

}  // namespace

class TrafficLightDetectorNode : public rclcpp::Node {
public:
  TrafficLightDetectorNode() : Node("traffic_light_detector") {
    // Camera input and region of interest.
    declare_parameter<std::string>("image_topic", "/OurCar/Sensors/RGBCameraLeft/image_raw");
    declare_parameter<double>("roi_top_fraction", 0.0);
    declare_parameter<double>("roi_bottom_fraction", 0.45);
    declare_parameter<double>("roi_left_fraction", 0.10);
    declare_parameter<double>("roi_right_fraction", 0.90);

    // Candidate shape and traffic-light housing filters.
    declare_parameter<int>("min_blob_area", 2);
    declare_parameter<int>("min_red_blob_area", 8);
    declare_parameter<double>("min_circularity", 0.55);
    declare_parameter<double>("min_color_fill_fraction", 0.45);
    declare_parameter<double>("min_yellow_housing_fraction", 0.10);
    declare_parameter<double>("max_yellow_housing_fraction", 0.65);

    // Temporal filtering and output behavior.
    declare_parameter<int>("debounce_frames", 3);
    declare_parameter<bool>("stop_on_yellow", false);
    declare_parameter<bool>("publish_debug_image", true);

    roi_top_fraction_ = get_parameter("roi_top_fraction").as_double();
    roi_bottom_fraction_ = get_parameter("roi_bottom_fraction").as_double();
    roi_left_fraction_ = get_parameter("roi_left_fraction").as_double();
    roi_right_fraction_ = get_parameter("roi_right_fraction").as_double();
    min_blob_area_ = get_parameter("min_blob_area").as_int();
    min_red_blob_area_ = get_parameter("min_red_blob_area").as_int();
    min_circularity_ = get_parameter("min_circularity").as_double();
    min_color_fill_fraction_ = get_parameter("min_color_fill_fraction").as_double();
    min_yellow_housing_fraction_ = get_parameter("min_yellow_housing_fraction").as_double();
    max_yellow_housing_fraction_ = get_parameter("max_yellow_housing_fraction").as_double();
    debounce_frames_ = std::max(1, static_cast<int>(get_parameter("debounce_frames").as_int()));
    stop_on_yellow_ = get_parameter("stop_on_yellow").as_bool();
    publish_debug_image_ = get_parameter("publish_debug_image").as_bool();

    state_pub_ = create_publisher<perception::msg::TrafficLightState>(
        "traffic_light/state", rclcpp::QoS(10));
    stop_pub_ = create_publisher<std_msgs::msg::Bool>(
        "traffic_light/must_stop", rclcpp::QoS(10));

    if (publish_debug_image_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
          "traffic_light/debug_image", rclcpp::SensorDataQoS());
    }

    // Match the bridge's best-effort image QoS.
    const auto image_topic = get_parameter("image_topic").as_string();
    image_sub_ = image_transport::create_subscription(
        this,
        image_topic,
        std::bind(&TrafficLightDetectorNode::OnImage, this, std::placeholders::_1),
        "raw",
        rclcpp::SensorDataQoS().get_rmw_qos_profile());

    RCLCPP_INFO(get_logger(), "Traffic light detector listening on '%s'", image_topic.c_str());
  }

private:
  // Detect, debounce, and publish one traffic-light observation.
  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    const cv::Mat &frame = cv_ptr->image;
    const int top = static_cast<int>(roi_top_fraction_ * frame.rows);
    const int bottom = std::clamp(
        static_cast<int>(roi_bottom_fraction_ * frame.rows),
        top + 1,
        frame.rows);

    const cv::Rect roi(0, top, frame.cols, bottom - top);
    const cv::Mat cropped = frame(roi);

    cv::Mat hsv;
    cv::cvtColor(cropped, hsv, cv::COLOR_BGR2HSV);

    cv::Mat housing_mask;
    cv::inRange(hsv, cv::Scalar(15, 80, 100), cv::Scalar(40, 255, 255), housing_mask);

    const Candidate red =
        BestCandidate(
            hsv, housing_mask, RedRange(),
            perception::msg::TrafficLightState::RED,
            min_red_blob_area_, true, true);
    const Candidate yellow =
        BestCandidate(
            hsv, housing_mask, YellowRange(),
            perception::msg::TrafficLightState::YELLOW,
            min_blob_area_, true, false);
    const Candidate green =
        BestCandidate(
            hsv, housing_mask, GreenRange(),
            perception::msg::TrafficLightState::GREEN,
            min_blob_area_, true, true);

    Candidate best;
    // Prefer valid red/green lamps before considering yellow highlights.
    for (const Candidate &c : {red, green}) {
      if (c.area > best.area) {
        best = c;
      }
    }

    if (best.area <= 0.0) {
      best = yellow;
    }

    const uint8_t raw_state =
        best.area > 0.0
            ? best.state
            : static_cast<uint8_t>(perception::msg::TrafficLightState::UNKNOWN);
    const uint8_t debounced_state = Debounce(raw_state);

    perception::msg::TrafficLightState state_msg;
    state_msg.header = msg->header;
    state_msg.state = debounced_state;
    state_msg.detected = best.area > 0.0;
    state_msg.pixel_area = static_cast<int32_t>(best.area);
    state_msg.confidence =
        best.area > 0.0
            ? static_cast<float>(std::clamp(best.circularity, 0.0, 1.0))
            : 0.0F;
    state_pub_->publish(state_msg);

    std_msgs::msg::Bool stop_msg;
    stop_msg.data = debounced_state == perception::msg::TrafficLightState::RED ||
                    (stop_on_yellow_ &&
                     debounced_state == perception::msg::TrafficLightState::YELLOW);
    stop_pub_->publish(stop_msg);

    if (publish_debug_image_) {
      PublishDebugImage(*msg, frame, roi, best);
    }
  }

  static ColorRange RedRange() {
    // Red wraps around hue 0/180 in OpenCV's 8-bit HSV, so it needs two bands.
    return {
        cv::Scalar(0, 120, 120),
        cv::Scalar(10, 255, 255),
        cv::Scalar(170, 120, 120),
        cv::Scalar(180, 255, 255),
        true,
    };
  }

  static ColorRange YellowRange() {
    return {
        cv::Scalar(18, 110, 120),
        cv::Scalar(35, 255, 255),
        cv::Scalar(0, 0, 0),
        cv::Scalar(0, 0, 0),
        false,
    };
  }

  static ColorRange GreenRange() {
    return {
        cv::Scalar(45, 80, 100),
        cv::Scalar(90, 255, 255),
        cv::Scalar(0, 0, 0),
        cv::Scalar(0, 0, 0),
        false,
    };
  }

  Candidate BestCandidate(
      const cv::Mat &hsv,
      const cv::Mat &housing_mask,
      const ColorRange &range,
      uint8_t state,
      int minimum_area,
      bool require_aligned_housing,
      bool allow_high_housing_fraction) const {
    cv::Mat mask;
    cv::inRange(hsv, range.low_a, range.high_a, mask);
    if (range.has_second_band) {
      cv::Mat mask_b;
      cv::inRange(hsv, range.low_b, range.high_b, mask_b);
      mask |= mask_b;
    }

    cv::morphologyEx(
        mask, mask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::morphologyEx(
        mask, mask, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    Candidate best;
    best.state = state;

    for (const auto &contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < minimum_area) {
        continue;
      }

      const double circularity = Circularity(contour, area);
      if (circularity < min_circularity_) {
        continue;
      }

      const cv::Rect bounds = cv::boundingRect(contour);
      const double center_fraction =
          static_cast<double>(bounds.x + bounds.width / 2) /
          static_cast<double>(std::max(1, hsv.cols));
      if (center_fraction < roi_left_fraction_ ||
          center_fraction > roi_right_fraction_) {
        continue;  // side-facing signal/building detail, not in the ego lane
      }

      if (ColorFillFraction(mask, bounds) < min_color_fill_fraction_) {
        continue;  // likely a hollow circular road sign rather than a filled lens
      }

      const double housing_fraction = YellowHousingSurroundFraction(hsv, bounds);
      if (housing_fraction < min_yellow_housing_fraction_ ||
          (!allow_high_housing_fraction && housing_fraction > max_yellow_housing_fraction_)) {
        continue;  // likely a vehicle lamp/sign, not a lens in the yellow signal housing
      }

      if (require_aligned_housing && !HasAlignedHousingStack(housing_mask, bounds)) {
        continue;  // nearby yellow pixels do not form the simulator's vertical three-lamp fixture
      }

      if (area > best.area) {
        best.area = area;
        best.circularity = circularity;
        best.bounds = bounds;
      }
    }

    return best;
  }

  uint8_t Debounce(uint8_t observed_state) {
    if (observed_state == pending_state_) {
      pending_count_ = std::min(pending_count_ + 1, debounce_frames_);
    } else {
      pending_state_ = observed_state;
      pending_count_ = 1;
    }

    if (pending_count_ >= debounce_frames_) {
      reported_state_ = pending_state_;
    }

    return reported_state_;
  }

  static cv::Scalar DebugColor(uint8_t state) {
    if (state == perception::msg::TrafficLightState::RED) {
      return cv::Scalar(0, 0, 255);
    }

    if (state == perception::msg::TrafficLightState::YELLOW) {
      return cv::Scalar(0, 255, 255);
    }

    return cv::Scalar(0, 255, 0);
  }

  void PublishDebugImage(
      const sensor_msgs::msg::Image &original_header_msg,
      const cv::Mat &frame,
      const cv::Rect &roi,
      const Candidate &best) {
    cv::Mat debug = frame.clone();
    cv::rectangle(debug, roi, cv::Scalar(255, 255, 255), 1);

    if (best.area > 0.0) {
      const cv::Rect box(
          best.bounds.x + roi.x,
          best.bounds.y + roi.y,
          best.bounds.width,
          best.bounds.height);
      cv::rectangle(debug, box, DebugColor(best.state), 2);
    }

    cv_bridge::CvImage out;
    out.header = original_header_msg.header;
    out.encoding = "bgr8";
    out.image = debug;
    debug_pub_->publish(*out.toImageMsg());
  }

  // Detector configuration.
  double roi_top_fraction_{0.0};
  double roi_bottom_fraction_{0.45};
  double roi_left_fraction_{0.10};
  double roi_right_fraction_{0.90};
  int min_blob_area_{2};
  int min_red_blob_area_{8};
  double min_circularity_{0.55};
  double min_color_fill_fraction_{0.45};
  double min_yellow_housing_fraction_{0.10};
  double max_yellow_housing_fraction_{0.65};
  int debounce_frames_{3};
  bool stop_on_yellow_{false};
  bool publish_debug_image_{true};

  // Debounce state.
  uint8_t pending_state_{perception::msg::TrafficLightState::UNKNOWN};
  int pending_count_{0};
  uint8_t reported_state_{perception::msg::TrafficLightState::UNKNOWN};

  // ROS interfaces.
  image_transport::Subscriber image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<perception::msg::TrafficLightState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrafficLightDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
