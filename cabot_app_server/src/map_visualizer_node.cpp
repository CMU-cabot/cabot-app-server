#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <people_msgs/msg/people.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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
    arrow_m_ = declare_parameter<double>("arrow_m", 1.0);
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
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/scan_matched_points2", scan_qos,
      std::bind(&MapVisualizer::pointCloudCallback, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&MapVisualizer::pathCallback, this, std::placeholders::_1));

    people_sub_ = create_subscription<people_msgs::msg::People>(
      "/people", 10, std::bind(&MapVisualizer::peopleCallback, this, std::placeholders::_1));

    image_pub_ = image_transport::create_publisher(this, "/rosmap_image");

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

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

    RCLCPP_DEBUG(get_logger(), "Processing point cloud with %u points", msg->width * msg->height);

    cv::Mat overlay = map_image_.clone();
    drawPeople(overlay, cv::Scalar(255, 0, 0), 0.7);
    drawPointCloud2(overlay, msg, cv::Scalar(255, 0, 255));
    auto robot_pixel = drawRobot(overlay, cv::Scalar(0, 0, 255));
    drawPath(overlay);

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
    image_pub_.publish(img_msg);
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

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_msg_ = msg;
  }

  void peopleCallback(const people_msgs::msg::People::SharedPtr msg)
  {
    people_msg_ = msg;
  }

  std::optional<cv::Point> drawRobot(cv::Mat & overlay, cv::Scalar color) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, base_frame_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
      return std::nullopt;
    }

    const auto [robot_world, robot_yaw] = transformToPose(transform);
    auto robot_pixel = worldToPixel(robot_world.first, robot_world.second);
    if (robot_pixel) {
      double head_x = robot_world.first + arrow_m_ * std::cos(robot_yaw);
      double head_y = robot_world.second + arrow_m_ * std::sin(robot_yaw);
      auto arrow_tip = worldToPixel(head_x, head_y);
      if (arrow_tip) {
        cv::arrowedLine(
          overlay, *robot_pixel, *arrow_tip, color, 5, cv::LINE_8, 0, arrow_m_ * 0.5);
      }
    }
    return robot_pixel;
  }

  void drawPointCloud2(
    cv::Mat & overlay,
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    cv::Scalar color)
  {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        map_frame_, msg->header.frame_id.empty() ? base_frame_ : msg->header.frame_id,
        msg->header.stamp, rclcpp::Duration::from_seconds(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      double lx = *iter_x;
      double ly = *iter_y;
      if (!std::isfinite(lx) || !std::isfinite(ly)) {
        continue;
      }
      auto world = transformPoint({lx, ly}, transform);
      auto pixel = worldToPixel(world.first, world.second);
      if (!pixel) {
        continue;
      }
      int x0 = std::max(pixel->x - 1, 0);
      int y0 = std::max(pixel->y - 1, 0);
      int x1 = std::min(pixel->x + 1, overlay.cols - 1);
      int y1 = std::min(pixel->y + 1, overlay.rows - 1);
      cv::rectangle(overlay, cv::Point(x0, y0), cv::Point(x1, y1), color, cv::FILLED);
    }
  }

  void drawPath(cv::Mat & overlay)
  {
    auto path = path_msg_;
    if (!path || path->poses.empty()) {
      return;
    }

    std::vector<cv::Point> points;
    points.reserve(path->poses.size());
    for (const auto & pose_stamped : path->poses) {
      const auto & pose_frame = pose_stamped.header.frame_id.empty() ?
        path->header.frame_id : pose_stamped.header.frame_id;
      geometry_msgs::msg::PoseStamped pose_in_map = pose_stamped;

      if (!pose_frame.empty() && pose_frame != map_frame_) {
        try {
          auto transform = tf_buffer_.lookupTransform(
            map_frame_, pose_frame, pose_stamped.header.stamp, rclcpp::Duration::from_seconds(0.1));
          tf2::doTransform(pose_stamped, pose_in_map, transform);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_DEBUG(get_logger(), "TF lookup for path failed: %s", ex.what());
          continue;
        }
      }

      auto pixel = worldToPixel(
        pose_in_map.pose.position.x,
        pose_in_map.pose.position.y);
      if (pixel) {
        points.push_back(*pixel);
      }
    }

    if (points.empty()) {
      return;
    }

    if (points.size() == 1) {
      cv::circle(overlay, points.front(), 1, cv::Scalar(255, 0, 0), cv::FILLED);
    } else {
      cv::polylines(overlay, points, false, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
    }
  }

  void drawPeople(cv::Mat & overlay, cv::Scalar color, double alpha)
  {
    auto people = people_msg_;
    if (!people || people->people.empty() || !map_msg_) {
      return;
    }

    const auto & people_frame = people->header.frame_id.empty() ? map_frame_ : people->header.frame_id;
    geometry_msgs::msg::TransformStamped transform;
    if (people_frame != map_frame_) {
      try {
        transform = tf_buffer_.lookupTransform(
          map_frame_, people_frame, people->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(get_logger(), "TF lookup for people failed: %s", ex.what());
        return;
      }
    } else {
      transform.header.stamp = people->header.stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = map_frame_;
      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation.w = 1.0;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
    }

    const double fill_alpha = alpha;
    const cv::Scalar fill_color = color;
    int radius_px = std::max(1, static_cast<int>(0.5 / map_msg_->info.resolution));
    for (const auto & person : people->people) {
      auto world = transformPoint({person.position.x, person.position.y}, transform);
      auto pixel = worldToPixel(world.first, world.second);
      if (pixel) {
        int x0 = std::max(pixel->x - radius_px, 0);
        int y0 = std::max(pixel->y - radius_px, 0);
        int x1 = std::min(pixel->x + radius_px, overlay.cols - 1);
        int y1 = std::min(pixel->y + radius_px, overlay.rows - 1);
        cv::Rect roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

        cv::Mat roi_view = overlay(roi);
        cv::Mat filled_roi = roi_view.clone();
        cv::Point local_center(pixel->x - roi.x, pixel->y - roi.y);
        cv::circle(filled_roi, local_center, radius_px, fill_color, cv::FILLED);
        cv::addWeighted(filled_roi, fill_alpha, roi_view, 1.0 - fill_alpha, 0.0, roi_view);
      }
    }
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
  nav_msgs::msg::Path::SharedPtr path_msg_;
  people_msgs::msg::People::SharedPtr people_msg_;
  cv::Mat map_image_;
  double origin_cos_{1.0};
  double origin_sin_{0.0};
  std::optional<rclcpp::Time> last_publish_time_;

  // ROS interfaces
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  image_transport::Publisher image_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapVisualizer>());
  rclcpp::shutdown();
  return 0;
}
