#include <chrono>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "diff_drive_robot/ml.h"
#include "diff_drive_robot/diff_drive_lib.h"

using namespace std::chrono_literals;

namespace diff_drive_robot
{
class LandmarksNode : public rclcpp::Node
{
public:
  LandmarksNode()
  : Node("landmarks_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter("distance_threshold", 0.08);
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("body_frame", "base_link");
    
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();
    std::string scan_topic = this->get_parameter("scan_topic").as_string();
    body_frame_ = this->get_parameter("body_frame").as_string();

    point_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/SLAM/points", 10);
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&LandmarksNode::lidarCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "LandmarksNode started listening on %s (dist_thresh=%.3f)",
      scan_topic.c_str(), distance_threshold_);
  }

private:
  double distance_threshold_{0.08};
  std::string body_frame_{"base_link"};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr point_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<double> range_data;
    range_data.reserve(msg->ranges.size());
    for (size_t i = 0; i < msg->ranges.size(); i++) {
      double r = msg->ranges[i];
      if (std::isnan(r) || std::isinf(r) || r < msg->range_min || r > msg->range_max) {
        range_data.push_back(0.0);
      } else {
        range_data.push_back(r);
      }
    }

    turtlelib::Clusters lidar = turtlelib::clustering(
      range_data, msg->angle_min, msg->angle_increment, distance_threshold_);
    std::vector<turtlelib::Vector2D> centroids = turtlelib::centroid_finder(lidar);
    turtlelib::ClustersCentroids cluster_points = turtlelib::shift_points(lidar, centroids);
    std::vector<turtlelib::Circle> detected_circles = turtlelib::circle_detection(cluster_points);
    std::vector<bool> is_circle = turtlelib::classification(detected_circles);

    try {
      const auto transform = tf_buffer_.lookupTransform(
        body_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(0.05));
      for (auto & circle : detected_circles) {
        geometry_msgs::msg::PointStamped source;
        geometry_msgs::msg::PointStamped target;
        source.header = msg->header;
        source.point.x = circle.a;
        source.point.y = circle.b;
        tf2::doTransform(source, target, transform);
        circle.a = target.point.x;
        circle.b = target.point.y;
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Cannot transform landmarks from %s to %s: %s",
        msg->header.frame_id.c_str(), body_frame_.c_str(), ex.what());
      return;
    }

    publishClusters(detected_circles, is_circle, body_frame_, msg->header.stamp);
  }

  void publishClusters(
    const std::vector<turtlelib::Circle> & detected_circles,
    const std::vector<bool> & is_circle,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray lidar_data;
    for (size_t i = 0; i < is_circle.size(); ++i) {
      if (!is_circle[i]) {
        continue;
      }

      visualization_msgs::msg::Marker obst;
      obst.header.frame_id = frame_id;
      obst.header.stamp = stamp;
      obst.ns = "detected_landmarks";
      obst.type = visualization_msgs::msg::Marker::CYLINDER;
      obst.scale.x = detected_circles[i].R * 2.0; // diameter
      obst.scale.y = detected_circles[i].R * 2.0;
      obst.scale.z = 0.25;
      obst.color.r = 0.1;
      obst.color.g = 0.8;
      obst.color.b = 0.2;
      obst.color.a = 1.0;
      obst.id = static_cast<int>(i);
      obst.lifetime = rclcpp::Duration(200ms);
      obst.action = visualization_msgs::msg::Marker::ADD;
      obst.pose.position.x = detected_circles[i].a;
      obst.pose.position.y = detected_circles[i].b;
      obst.pose.orientation.w = 1.0;
      lidar_data.markers.push_back(obst);
    }

    point_publisher_->publish(lidar_data);
  }
};
}  // namespace diff_drive_robot

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diff_drive_robot::LandmarksNode>());
  rclcpp::shutdown();
  return 0;
}
