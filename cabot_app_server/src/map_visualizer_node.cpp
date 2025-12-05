#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
double quaternionYaw(const geometry_msgs::msg::Quaternion & q)
{
  return tf2::getYaw(q);
}
}  // namespace

class MapVisualizer : public rclcpp::Node
{
public:
  MapVisualizer()
  : Node("map_visualizer_cpp"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    range_ring_m_ = declare_parameter<double>("range_ring_m", 0.2);
    arrow_m_ = declare_parameter<double>("arrow_m", 0.7);
    crop_radius_m_ = declare_parameter<double>("crop_radius_m", 15.0);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 50);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);
    if (publish_rate_hz_ > 0.0) {
      publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_);
    }

    rclcpp::QoS map_qos(10);
    map_qos.transient_local();
    map_qos.reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos, std::bind(&MapVisualizer::mapCallback, this, std::placeholders::_1));

    auto scan_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", scan_qos, std::bind(&MapVisualizer::scanCallback, this, std::placeholders::_1));

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("/viz_image", 1);

    RCLCPP_INFO(get_logger(), "map_visualizer_cpp node started");
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_msg_ = msg;
    const auto & info = msg->info;
    cv::Mat img(info.height, info.width, CV_8UC3, cv::Scalar(120, 120, 120));
    const std::vector<int8_t> & data = msg->data;
    for (size_t idx = 0; idx < data.size(); ++idx) {
      const int8_t val = data[idx];
      if (val == 0) {
        img.data[idx * 3 + 0] = 240;
        img.data[idx * 3 + 1] = 240;
        img.data[idx * 3 + 2] = 240;
      } else if (val > occupied_threshold_) {
        img.data[idx * 3 + 0] = 0;
        img.data[idx * 3 + 1] = 0;
        img.data[idx * 3 + 2] = 0;
      }
    }
    cv::flip(img, map_image_, 0);

    double yaw = quaternionYaw(info.origin.orientation);
    origin_cos_ = std::cos(yaw);
    origin_sin_ = std::sin(yaw);

    RCLCPP_INFO(
      get_logger(), "Received map %u x %u, res=%.3f m/px",
      info.width, info.height, info.resolution);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!map_msg_ || map_image_.empty()) {
      return;
    }
    auto now = this->get_clock()->now();
    if (publish_period_ && last_publish_time_) {
      if ((now - *last_publish_time_) < *publish_period_) {
        return;
      }
    }

    RCLCPP_DEBUG(get_logger(), "Processing scan with %zu ranges", msg->ranges.size());
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, msg->header.frame_id.empty() ? base_frame_ : msg->header.frame_id,
        msg->header.stamp, rclcpp::Duration::from_seconds(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    cv::Mat overlay = map_image_.clone();

    const auto [robot_world, robot_yaw] = transformToPose(transform);
    int ring_radius = std::max(1, static_cast<int>(range_ring_m_ / map_msg_->info.resolution));

    const auto & ranges = msg->ranges;
    for (size_t idx = 0; idx < ranges.size(); ++idx) {
      const float rng = ranges[idx];
      if (!std::isfinite(rng) || rng <= 0.0f || rng > msg->range_max) {
        continue;
      }
      double angle = msg->angle_min + static_cast<double>(idx) * msg->angle_increment;
      double lx = rng * std::cos(angle);
      double ly = rng * std::sin(angle);
      auto world = transformPoint({lx, ly}, transform);
      auto pixel = worldToPixel(world.first, world.second);
      if (!pixel) {
        continue;
      }
      cv::circle(overlay, *pixel, 1, cv::Scalar(255, 0, 255), cv::FILLED);
    }

    auto robot_pixel = worldToPixel(robot_world.first, robot_world.second);
    if (robot_pixel) {
      cv::circle(overlay, *robot_pixel, ring_radius, cv::Scalar(255, 0, 0), 2);

      double head_x = robot_world.first + arrow_m_ * std::cos(robot_yaw);
      double head_y = robot_world.second + arrow_m_ * std::sin(robot_yaw);
      auto arrow_tip = worldToPixel(head_x, head_y);
      if (arrow_tip) {
        cv::arrowedLine(
          overlay, *robot_pixel, *arrow_tip, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.25);
      }
    }

    cv::Mat output = overlay;
    if (robot_pixel && crop_radius_m_ > 0.0) {
      int radius_px = std::max(1, static_cast<int>(crop_radius_m_ / map_msg_->info.resolution));
      int x0 = std::max(robot_pixel->x - radius_px, 0);
      int y0 = std::max(robot_pixel->y - radius_px, 0);
      int x1 = std::min(robot_pixel->x + radius_px, overlay.cols - 1);
      int y1 = std::min(robot_pixel->y + radius_px, overlay.rows - 1);
      cv::Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
      output = overlay(roi).clone();
    }

    auto img_msg = cv_bridge::CvImage(msg->header, "bgr8", output).toImageMsg();
    img_msg->header.frame_id = map_frame_;
    image_pub_->publish(*img_msg);
    last_publish_time_ = now;
  }

  std::pair<std::pair<double, double>, double> transformToPose(
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    double yaw = quaternionYaw(transform.transform.rotation);
    return {{transform.transform.translation.x, transform.transform.translation.y}, yaw};
  }

  std::pair<double, double> transformPoint(
    const std::pair<double, double> & point,
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    double yaw = quaternionYaw(transform.transform.rotation);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    double x = point.first;
    double y = point.second;
    double rx = cos_yaw * x - sin_yaw * y + transform.transform.translation.x;
    double ry = sin_yaw * x + cos_yaw * y + transform.transform.translation.y;
    return {rx, ry};
  }

  std::optional<cv::Point> worldToPixel(double wx, double wy) const
  {
    if (!map_msg_) {
      return std::nullopt;
    }
    double dx = wx - map_msg_->info.origin.position.x;
    double dy = wy - map_msg_->info.origin.position.y;
    double mx = (origin_cos_ * dx + origin_sin_ * dy) / map_msg_->info.resolution;
    double my = (-origin_sin_ * dx + origin_cos_ * dy) / map_msg_->info.resolution;

    int ix = static_cast<int>(std::floor(mx));
    int iy = static_cast<int>(std::floor(my));

    if (ix < 0 || iy < 0 || ix >= static_cast<int>(map_msg_->info.width) ||
      iy >= static_cast<int>(map_msg_->info.height))
    {
      return std::nullopt;
    }

    int image_y = static_cast<int>(map_msg_->info.height) - 1 - iy;
    return cv::Point(ix, image_y);
  }

  // parameters
  std::string map_frame_;
  std::string base_frame_;
  double range_ring_m_;
  double arrow_m_;
  double crop_radius_m_;
  int occupied_threshold_;
  double publish_rate_hz_;
  std::optional<rclcpp::Duration> publish_period_;

  // state
  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  cv::Mat map_image_;
  double origin_cos_{1.0};
  double origin_sin_{0.0};
  std::optional<rclcpp::Time> last_publish_time_;

  // ROS interfaces
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapVisualizer>());
  rclcpp::shutdown();
  return 0;
}
