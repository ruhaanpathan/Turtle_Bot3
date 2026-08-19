// Copyright 2017-2026 EAI TEAM and YDLIDAR ROS 2 driver contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>

#include "geometry_msgs/msg/point32.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_srvs/srv/empty.hpp"
#include "src/CYdLidar.h"
#include "ydlidar_ros2_driver/scan_utils.hpp"

#ifndef YDLIDAR_ROS2_DRIVER_VERSION
#define YDLIDAR_ROS2_DRIVER_VERSION "unknown"
#endif

namespace
{

struct DriverParameters
{
  std::string port;
  std::string frame_id;
  std::string ignore_array;
  int baudrate;
  int lidar_type;
  int device_type;
  int sample_rate;
  int abnormal_check_count;
  int intensity_bit;
  bool fixed_resolution;
  bool reversion;
  bool inverted;
  bool auto_reconnect;
  bool single_channel;
  bool intensity;
  bool auto_intensity;
  bool support_motor_dtr;
  bool debug;
  float angle_max;
  float angle_min;
  float range_max;
  float range_min;
  float frequency;
  bool invalid_range_is_inf;
  int m1_mode;
  int m2_mode;
  int m3_mode;
};

DriverParameters declare_parameters(const rclcpp::Node::SharedPtr & node)
{
  DriverParameters parameters;
  parameters.port = node->declare_parameter<std::string>("port", "/dev/ydlidar");
  parameters.frame_id = node->declare_parameter<std::string>("frame_id", "laser_frame");
  parameters.ignore_array = node->declare_parameter<std::string>("ignore_array", "");
  parameters.baudrate = node->declare_parameter<int>("baudrate", 115200);
  parameters.lidar_type = node->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
  parameters.device_type = node->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
  parameters.sample_rate = node->declare_parameter<int>("sample_rate", 3);
  parameters.abnormal_check_count = node->declare_parameter<int>("abnormal_check_count", 4);
  parameters.intensity_bit = node->declare_parameter<int>("intensity_bit", 0);
  parameters.fixed_resolution = node->declare_parameter<bool>("fixed_resolution", false);
  parameters.reversion = node->declare_parameter<bool>("reversion", false);
  parameters.inverted = node->declare_parameter<bool>("inverted", false);
  parameters.auto_reconnect = node->declare_parameter<bool>("auto_reconnect", true);
  parameters.single_channel = node->declare_parameter<bool>("isSingleChannel", true);
  parameters.intensity = node->declare_parameter<bool>("intensity", false);
  parameters.auto_intensity = node->declare_parameter<bool>("auto_intensity", false);
  parameters.support_motor_dtr = node->declare_parameter<bool>("support_motor_dtr", true);
  parameters.debug = node->declare_parameter<bool>("debug", false);
  parameters.angle_max = static_cast<float>(
    node->declare_parameter<double>("angle_max", 180.0));
  parameters.angle_min = static_cast<float>(
    node->declare_parameter<double>("angle_min", -180.0));
  parameters.range_max = static_cast<float>(
    node->declare_parameter<double>("range_max", 8.0));
  parameters.range_min = static_cast<float>(
    node->declare_parameter<double>("range_min", 0.12));
  parameters.frequency = static_cast<float>(
    node->declare_parameter<double>("frequency", 7.0));
  parameters.invalid_range_is_inf =
    node->declare_parameter<bool>("invalid_range_is_inf", false);
  parameters.m1_mode = node->declare_parameter<int>("m1_mode", 0);
  parameters.m2_mode = node->declare_parameter<int>("m2_mode", 0);
  parameters.m3_mode = node->declare_parameter<int>("m3_mode", 1);
  return parameters;
}

bool validate_parameters(
  const DriverParameters & parameters, const rclcpp::Logger & logger)
{
  if (parameters.port.empty() || parameters.frame_id.empty()) {
    RCLCPP_FATAL(logger, "Parameters 'port' and 'frame_id' must not be empty");
    return false;
  }
  if (parameters.baudrate <= 0 || parameters.sample_rate <= 0 ||
    parameters.abnormal_check_count <= 0)
  {
    RCLCPP_FATAL(
      logger, "baudrate, sample_rate, and abnormal_check_count must be positive");
    return false;
  }
  if (!std::isfinite(parameters.angle_min) || !std::isfinite(parameters.angle_max) ||
    parameters.angle_min >= parameters.angle_max)
  {
    RCLCPP_FATAL(logger, "angle_min and angle_max must be finite, with angle_min < angle_max");
    return false;
  }
  if (!std::isfinite(parameters.range_min) || !std::isfinite(parameters.range_max) ||
    parameters.range_min < 0.0F || parameters.range_min >= parameters.range_max)
  {
    RCLCPP_FATAL(logger, "range_min and range_max must be finite, with 0 <= min < max");
    return false;
  }
  if (!std::isfinite(parameters.frequency) || parameters.frequency <= 0.0F) {
    RCLCPP_FATAL(logger, "frequency must be finite and positive");
    return false;
  }
  return true;
}

template<typename ValueT>
bool set_option(
  CYdLidar & laser, const int property, const ValueT & value,
  const char * name, const rclcpp::Logger & logger)
{
  if (laser.setlidaropt(property, &value, sizeof(ValueT))) {
    return true;
  }
  RCLCPP_FATAL(logger, "YDLidar SDK rejected parameter '%s'", name);
  return false;
}

bool set_string_option(
  CYdLidar & laser, const int property, const std::string & value,
  const char * name, const rclcpp::Logger & logger)
{
  if (laser.setlidaropt(property, value.c_str(), static_cast<int>(value.size()))) {
    return true;
  }
  RCLCPP_FATAL(logger, "YDLidar SDK rejected parameter '%s'", name);
  return false;
}

bool configure_laser(
  CYdLidar & laser, const DriverParameters & p, const rclcpp::Logger & logger)
{
  // SDK 1.2.19 does not initialize CYdLidar::m_AutoIntensity. Set it before
  // initialize() so X2 startup never depends on indeterminate memory.
  laser.setAutoIntensity(p.auto_intensity);
  laser.setEnableDebug(p.debug);

  return
    set_string_option(laser, LidarPropSerialPort, p.port, "port", logger) &&
    set_string_option(laser, LidarPropIgnoreArray, p.ignore_array, "ignore_array", logger) &&
    set_option(laser, LidarPropSerialBaudrate, p.baudrate, "baudrate", logger) &&
    set_option(laser, LidarPropLidarType, p.lidar_type, "lidar_type", logger) &&
    set_option(laser, LidarPropDeviceType, p.device_type, "device_type", logger) &&
    set_option(laser, LidarPropSampleRate, p.sample_rate, "sample_rate", logger) &&
    set_option(
    laser, LidarPropAbnormalCheckCount, p.abnormal_check_count,
    "abnormal_check_count", logger) &&
    set_option(laser, LidarPropIntenstiyBit, p.intensity_bit, "intensity_bit", logger) &&
    set_option(
    laser, LidarPropFixedResolution, p.fixed_resolution, "fixed_resolution", logger) &&
    set_option(laser, LidarPropReversion, p.reversion, "reversion", logger) &&
    set_option(laser, LidarPropInverted, p.inverted, "inverted", logger) &&
    set_option(laser, LidarPropAutoReconnect, p.auto_reconnect, "auto_reconnect", logger) &&
    set_option(
    laser, LidarPropSingleChannel, p.single_channel, "isSingleChannel", logger) &&
    set_option(laser, LidarPropIntenstiy, p.intensity, "intensity", logger) &&
    set_option(
    laser, LidarPropSupportMotorDtrCtrl, p.support_motor_dtr,
    "support_motor_dtr", logger) &&
    set_option(laser, LidarPropMaxAngle, p.angle_max, "angle_max", logger) &&
    set_option(laser, LidarPropMinAngle, p.angle_min, "angle_min", logger) &&
    set_option(laser, LidarPropMaxRange, p.range_max, "range_max", logger) &&
    set_option(laser, LidarPropMinRange, p.range_min, "range_min", logger) &&
    set_option(laser, LidarPropScanFrequency, p.frequency, "frequency", logger);
}

const char * driver_error(const CYdLidar & laser)
{
  return ydlidar::core::common::DriverInterface::DescribeDriverError(
    laser.getDriverError());
}

void log_startup_configuration(
  const DriverParameters & p, const rclcpp::Logger & logger)
{
  RCLCPP_INFO(
    logger,
    "YDLidar profile: port=%s baud=%d lidar_type=%d device_type=%d sample_rate=%d "
    "single_channel=%s intensity=%s auto_intensity=%s motor_dtr=%s auto_reconnect=%s",
    p.port.c_str(), p.baudrate, p.lidar_type, p.device_type, p.sample_rate,
    p.single_channel ? "true" : "false", p.intensity ? "true" : "false",
    p.auto_intensity ? "true" : "false", p.support_motor_dtr ? "true" : "false",
    p.auto_reconnect ? "true" : "false");
}

bool publish_scan(
  const LaserScan & scan, const DriverParameters & parameters,
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr & laser_publisher,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr & cloud_publisher)
{
  std::size_t bin_count = 0U;
  if (!ydlidar_ros2_driver::calculate_scan_bin_count(
      scan.config.min_angle, scan.config.max_angle, scan.config.angle_increment, bin_count))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Discarding scan with invalid geometry: min=%g max=%g increment=%g",
      scan.config.min_angle, scan.config.max_angle, scan.config.angle_increment);
    return false;
  }

  sensor_msgs::msg::LaserScan scan_message;
  sensor_msgs::msg::PointCloud cloud_message;
  if (scan.stamp == 0U) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 10000,
      "SDK supplied a zero timestamp; using the ROS clock");
    scan_message.header.stamp = node->now();
  } else {
    constexpr uint64_t nanoseconds_per_second = 1000000000ULL;
    scan_message.header.stamp.sec = static_cast<int32_t>(
      scan.stamp / nanoseconds_per_second);
    scan_message.header.stamp.nanosec = static_cast<uint32_t>(
      scan.stamp % nanoseconds_per_second);
  }
  scan_message.header.frame_id = parameters.frame_id;
  cloud_message.header = scan_message.header;
  scan_message.angle_min = scan.config.min_angle;
  scan_message.angle_increment = scan.config.angle_increment;
  scan_message.angle_max = ydlidar_ros2_driver::scan_angle_max(
    scan_message.angle_min, scan_message.angle_increment, bin_count);
  scan_message.scan_time = scan.config.scan_time;
  scan_message.time_increment = scan.config.time_increment;
  scan_message.range_min = scan.config.min_range;
  scan_message.range_max = scan.config.max_range;
  scan_message.ranges.assign(
    bin_count, ydlidar_ros2_driver::invalid_scan_range(parameters.invalid_range_is_inf));
  scan_message.intensities.assign(bin_count, 0.0F);

  cloud_message.channels.resize(2U);
  cloud_message.channels[0].name = "intensities";
  cloud_message.channels[1].name = "stamps";

  for (std::size_t i = 0U; i < scan.points.size(); ++i) {
    const auto & sample = scan.points[i];
    if (!std::isfinite(sample.angle) ||
      !ydlidar_ros2_driver::valid_scan_range(
        sample.range, scan.config.min_range, scan.config.max_range))
    {
      continue;
    }

    std::size_t index = 0U;
    if (ydlidar_ros2_driver::scan_bin_index(
        sample.angle, scan.config.min_angle, scan.config.angle_increment,
        bin_count, index))
    {
      const float existing_range = scan_message.ranges[index];
      if (!ydlidar_ros2_driver::valid_scan_range(
          existing_range, scan.config.min_range, scan.config.max_range) ||
        sample.range < existing_range)
      {
        scan_message.ranges[index] = sample.range;
        scan_message.intensities[index] = sample.intensity;
      }
    }

    geometry_msgs::msg::Point32 point;
    point.x = sample.range * std::cos(sample.angle);
    point.y = sample.range * std::sin(sample.angle);
    point.z = 0.0F;
    cloud_message.points.push_back(point);
    cloud_message.channels[0].values.push_back(sample.intensity);
    cloud_message.channels[1].values.push_back(
      static_cast<float>(i) * scan.config.time_increment);
  }

  laser_publisher->publish(scan_message);
  cloud_publisher->publish(cloud_message);
  return true;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");
  const auto logger = node->get_logger();
  RCLCPP_INFO(
    logger, "[YDLIDAR INFO] Current ROS Driver Version: %s",
    YDLIDAR_ROS2_DRIVER_VERSION);

  const DriverParameters parameters = declare_parameters(node);
  if (!validate_parameters(parameters, logger)) {
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  CYdLidar laser;
  if (!configure_laser(laser, parameters, logger)) {
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  log_startup_configuration(parameters, logger);

  if (!laser.initialize()) {
    RCLCPP_FATAL(
      logger, "Failed to initialize YDLidar: driver=%s; transport=%s",
      driver_error(laser), laser.DescribeError());
    laser.disconnecting();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  if (parameters.lidar_type == TYPE_GS) {
    if (!laser.setWorkMode(parameters.m1_mode, 0x01) ||
      !laser.setWorkMode(parameters.m2_mode, 0x02) ||
      !laser.setWorkMode(parameters.m3_mode, 0x04))
    {
      RCLCPP_FATAL(logger, "Failed to configure GS-series work modes");
      laser.disconnecting();
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
  }

  if (!laser.turnOn()) {
    RCLCPP_FATAL(
      logger,
      "Failed to start YDLidar: driver=%s; transport=%s. The port opened, but no valid "
      "scan revolution arrived. Check that the head spins, verify 5 V under load, and "
      "inspect the LiDAR TX-to-adapter RX connection.",
      driver_error(laser), laser.DescribeError());
    laser.turnOff();
    laser.disconnecting();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const auto laser_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS());
  const auto cloud_publisher = node->create_publisher<sensor_msgs::msg::PointCloud>(
    "point_cloud", rclcpp::SensorDataQoS());

  bool scanning = true;
  const auto stop_service = node->create_service<std_srvs::srv::Empty>(
    "stop_scan",
    [&laser, &scanning, logger](
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      if (laser.turnOff()) {
        scanning = false;
        RCLCPP_INFO(logger, "YDLidar scanning stopped");
      } else {
        RCLCPP_ERROR(logger, "Failed to stop YDLidar scanning");
      }
    });
  const auto start_service = node->create_service<std_srvs::srv::Empty>(
    "start_scan",
    [&laser, &scanning, logger](
      const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      if (laser.turnOn()) {
        scanning = true;
        RCLCPP_INFO(logger, "YDLidar scanning started");
      } else {
        RCLCPP_ERROR(
          logger, "Failed to restart YDLidar: driver=%s; transport=%s",
          driver_error(laser), laser.DescribeError());
      }
    });
  (void)stop_service;
  (void)start_service;

  rclcpp::WallRate idle_rate(20.0);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (!scanning) {
      idle_rate.sleep();
      continue;
    }

    LaserScan scan;
    if (!laser.doProcessSimple(scan)) {
      RCLCPP_ERROR_THROTTLE(
        logger, *node->get_clock(), 5000,
        "Failed to get scan: driver=%s; transport=%s",
        driver_error(laser), laser.DescribeError());
      idle_rate.sleep();
      continue;
    }
    publish_scan(scan, parameters, node, laser_publisher, cloud_publisher);
  }

  RCLCPP_INFO(logger, "[YDLIDAR INFO] Stopping YDLidar");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
