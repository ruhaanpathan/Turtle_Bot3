#include <algorithm>
#include <functional>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "diff_drive_robot/diff_drive_lib.h"
#include "diff_drive_robot/ekf.h"

namespace diff_drive_robot
{
class EkfSlamNode : public rclcpp::Node
{
public:
  EkfSlamNode()
  : Node("ekf_slam_node")
  {
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("body_frame", "base_link");
    this->declare_parameter("publish_tf", false);
    this->declare_parameter("wheel_radius", 0.033);
    this->declare_parameter("wheel_separation", 0.170);
    this->declare_parameter("process_covariance", 0.01);
    this->declare_parameter("sensor_covariance", 0.001);
    this->declare_parameter("max_obstacles", 15);
    this->declare_parameter("dk_threshold", 1.0);

    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    body_frame_ = this->get_parameter("body_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    double radius = this->get_parameter("wheel_radius").as_double();
    double separation = this->get_parameter("wheel_separation").as_double();
    process_covariance_ = this->get_parameter("process_covariance").as_double();
    sensor_covariance_ = this->get_parameter("sensor_covariance").as_double();
    max_obstacles_ = this->get_parameter("max_obstacles").as_int();
    dk_threshold_ = this->get_parameter("dk_threshold").as_double();
    if (map_frame_.empty() || odom_frame_.empty() || body_frame_.empty() ||
        radius <= 0.0 || separation <= 0.0 || process_covariance_ < 0.0 ||
        sensor_covariance_ <= 0.0 || max_obstacles_ <= 0 || dk_threshold_ <= 0.0)
    {
      throw std::invalid_argument("EKF SLAM frames, geometry and covariance parameters are invalid");
    }

    tbot_ = turtlelib::DiffDrive(separation, radius);
    ekf_ = std::make_unique<turtlelib::EKF>(tbot_.q, max_obstacles_, process_covariance_, sensor_covariance_);

    // Publishers
    slam_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/slam_odom", 10);
    slam_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/slam_map", 10);

    // Subscribers
    js_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&EkfSlamNode::jointStateCallback, this, std::placeholders::_1));

    landmark_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      "/SLAM/points", 10,
      std::bind(&EkfSlamNode::landmarkCallback, this, std::placeholders::_1));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(this->get_logger(),
      "Experimental EkfSlamNode initialized — map=%s odom=%s base=%s "
      "max_obst=%d dk_thresh=%.2f publish_tf=%s",
      map_frame_.c_str(), odom_frame_.c_str(), body_frame_.c_str(),
      max_obstacles_, dk_threshold_, publish_tf_ ? "true" : "false");
  }

private:
  std::string map_frame_;
  std::string odom_frame_;
  std::string body_frame_;
  bool publish_tf_{false};
  double process_covariance_;
  double sensor_covariance_;
  int max_obstacles_{15};
  double dk_threshold_{1.0};

  turtlelib::DiffDrive tbot_{0.0, 0.0};
  std::unique_ptr<turtlelib::EKF> ekf_;

  bool obstacles_initialized_{false};
  int num_landmarks_known_{0};
  bool first_js_{true};
  double prev_phi_l_{0.0};
  double prev_phi_r_{0.0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr slam_odom_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slam_map_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_subscriber_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::size_t left_index = 0;
    std::size_t right_index = 1;
    if (!msg->name.empty()) {
      const auto left = std::find(msg->name.begin(), msg->name.end(), "left_wheel_joint");
      const auto right = std::find(msg->name.begin(), msg->name.end(), "right_wheel_joint");
      if (left == msg->name.end() || right == msg->name.end()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "JointState is missing left_wheel_joint or right_wheel_joint");
        return;
      }
      left_index = static_cast<std::size_t>(std::distance(msg->name.begin(), left));
      right_index = static_cast<std::size_t>(std::distance(msg->name.begin(), right));
    }
    if (msg->position.size() <= std::max(left_index, right_index)) return;

    const double phi_l = msg->position[left_index];
    const double phi_r = msg->position[right_index];
    if (!std::isfinite(phi_l) || !std::isfinite(phi_r)) return;

    if (first_js_) {
      prev_phi_l_ = phi_l;
      prev_phi_r_ = phi_r;
      tbot_.phi_l = prev_phi_l_;
      tbot_.phi_r = prev_phi_r_;
      first_js_ = false;
      return;
    }

    double dphi_l = phi_l - prev_phi_l_;
    double dphi_r = phi_r - prev_phi_r_;
    prev_phi_l_ = phi_l;
    prev_phi_r_ = phi_r;
    if (std::abs(dphi_l) > 2.0 * turtlelib::PI ||
        std::abs(dphi_r) > 2.0 * turtlelib::PI)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring implausible wheel-position jump (left=%.3f rad, right=%.3f rad)",
        dphi_l, dphi_r);
      return;
    }

    // Forward kinematics to update uncorrected odometry pose q
    tbot_.forward_kin(dphi_l, dphi_r);

    // EKF prediction update
    ekf_->prediction(tbot_.q);

    publishMapToOdomTf(msg->header.stamp);
  }

  void landmarkCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    std::vector<bool> associated_this_scan(max_obstacles_, false);
    for (size_t i = 0; i < msg->markers.size(); i++) {
      if (msg->markers[i].action == visualization_msgs::msg::Marker::ADD &&
          msg->markers[i].color.a > 0.0)
      {
        if (msg->markers[i].header.frame_id != body_frame_) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Ignoring landmark in frame '%s'; expected '%s'",
            msg->markers[i].header.frame_id.c_str(), body_frame_.c_str());
          continue;
        }
        turtlelib::Circle lmark = {
          msg->markers[i].pose.position.x,
          msg->markers[i].pose.position.y,
          msg->markers[i].scale.x / 2.0  // radius
        };

        int l = -1;
        double d_star = std::numeric_limits<double>::infinity();
        bool initialized_now = false;

        if (num_landmarks_known_ == 0) {
          ekf_->initialization(0, msg->markers[i].pose.position.x, msg->markers[i].pose.position.y);
          ekf_->izd.at(0) = true;
          ekf_->obst_radii.at(0) = msg->markers[i].scale.x;
          obstacles_initialized_ = true;
          num_landmarks_known_++;
          l = 0;
          initialized_now = true;
        } else {
          for (int k = 0; k < num_landmarks_known_; k++) {
            if (associated_this_scan.at(k)) continue;
            const double d_k = ekf_->mah_distance(lmark, k);
            if (std::isfinite(d_k) && d_k < d_star) {
              l = k;
              d_star = d_k;
            }
          }

          if (d_star > dk_threshold_) {
            if (num_landmarks_known_ >= max_obstacles_) {
              continue;
            }
            l = num_landmarks_known_;
            ekf_->initialization(l, msg->markers[i].pose.position.x, msg->markers[i].pose.position.y);
            ekf_->izd.at(l) = true;
            obstacles_initialized_ = true;
            ekf_->obst_radii.at(l) = msg->markers[i].scale.x;
            num_landmarks_known_++;
            initialized_now = true;
          }
        }

        if (!initialized_now && l >= 0 && l < max_obstacles_ && ekf_->izd.at(l)) {
          ekf_->correction(l, msg->markers[i].pose.position.x, msg->markers[i].pose.position.y);
        }
        if (l >= 0 && l < max_obstacles_) {
          associated_this_scan.at(l) = true;
        }
      }
    }

    if (obstacles_initialized_) {
      slam_map_publisher_->publish(getObstacleMarkers(this->now()));
      rclcpp::Time stamp = msg->markers.empty() ? this->now() : rclcpp::Time(msg->markers[0].header.stamp);
      publishMapToOdomTf(stamp);
    }
  }

  void publishMapToOdomTf(const rclcpp::Time & stamp)
  {
    // EKF state estimate for robot pose (in map frame)
    double map_x = ekf_->zeta_est(1);
    double map_y = ekf_->zeta_est(2);
    double map_theta = ekf_->zeta_est(0);

    // Uncorrected wheel odometry pose (in odom frame)
    double odom_x = tbot_.q.x;
    double odom_y = tbot_.q.y;
    double odom_theta = tbot_.q.theta;

    // T_map_base = [cos(mth) -sin(mth) mx; sin(mth) cos(mth) my; 0 0 1]
    // T_odom_base = [cos(oth) -sin(oth) ox; sin(oth) cos(oth) oy; 0 0 1]
    // T_map_odom = T_map_base * (T_odom_base)^-1
    turtlelib::Transform2D T_map_base({map_x, map_y}, map_theta);
    turtlelib::Transform2D T_odom_base({odom_x, odom_y}, odom_theta);
    turtlelib::Transform2D T_map_odom = T_map_base * T_odom_base.inv();

    tf2::Quaternion q_mo;
    q_mo.setRPY(0.0, 0.0, T_map_odom.rotation());

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_msg.transform.translation.x = T_map_odom.translation().x;
    tf_msg.transform.translation.y = T_map_odom.translation().y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = tf2::toMsg(q_mo);

    if (publish_tf_ && tf_broadcaster_) {
      tf_broadcaster_->sendTransform(tf_msg);
    }

    // Also publish SLAM odometry message
    nav_msgs::msg::Odometry slam_odom;
    slam_odom.header.stamp = stamp;
    slam_odom.header.frame_id = map_frame_;
    slam_odom.child_frame_id = body_frame_;

    tf2::Quaternion q_map;
    q_map.setRPY(0.0, 0.0, map_theta);
    slam_odom.pose.pose.position.x = map_x;
    slam_odom.pose.pose.position.y = map_y;
    slam_odom.pose.pose.orientation = tf2::toMsg(q_map);

    slam_odom_publisher_->publish(slam_odom);
  }

  visualization_msgs::msg::MarkerArray getObstacleMarkers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray all_obst;
    for (int i = 0; i < max_obstacles_; ++i) {
      if (ekf_->izd.at(i)) {
        visualization_msgs::msg::Marker obst;
        obst.header.frame_id = map_frame_;
        obst.header.stamp = stamp;
        obst.type = visualization_msgs::msg::Marker::CYLINDER;
        obst.scale.x = ekf_->obst_radii.at(i);
        obst.scale.y = ekf_->obst_radii.at(i);
        obst.scale.z = 0.25;
        obst.color.a = 1.0;
        obst.color.g = 1.0;
        obst.color.r = 0.0;
        obst.color.b = 0.0;
        obst.id = i;
        obst.ns = "ekf_landmarks";
        obst.pose.position.x = ekf_->zeta_est(3 + (2 * i));
        obst.pose.position.y = ekf_->zeta_est(4 + (2 * i));
        obst.pose.orientation.w = 1.0;
        obst.action = visualization_msgs::msg::Marker::ADD;

        all_obst.markers.push_back(obst);
      }
    }
    return all_obst;
  }
};
}  // namespace diff_drive_robot

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diff_drive_robot::EkfSlamNode>());
  rclcpp::shutdown();
  return 0;
}
