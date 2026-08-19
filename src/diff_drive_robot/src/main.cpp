#include <algorithm>
#include <memory>
#include <string>
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <lgpio.h>

#include "diff_drive_robot/motor_driver.h"
#include "diff_drive_robot/encoder.h"
#include "diff_drive_robot/pid.h"
#include "diff_drive_robot/odometry.h"

using namespace std::chrono_literals;

namespace diff_drive_robot
{

// ============================================================================
// EncoderNode
// ============================================================================
class EncoderNode : public rclcpp::Node
{
public:
  EncoderNode(int chip_handle, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("encoder_node", options), chip_(chip_handle)
  {
    // Declare parameters
    this->declare_parameter("left_encoder_a", 17);
    this->declare_parameter("left_encoder_b", 27);
    this->declare_parameter("right_encoder_a", 22);
    this->declare_parameter("right_encoder_b", 26);
    this->declare_parameter("left_encoder_reverse", false);
    this->declare_parameter("right_encoder_reverse", false);
    this->declare_parameter("encoder_ppr", 11);
    this->declare_parameter("gear_ratio", 30);
    this->declare_parameter("publish_rate", 50.0);
    this->declare_parameter("poll_rate", 20000.0);  // encoder GPIO polling frequency

    // Read parameters
    int left_a = this->get_parameter("left_encoder_a").as_int();
    int left_b = this->get_parameter("left_encoder_b").as_int();
    int right_a = this->get_parameter("right_encoder_a").as_int();
    int right_b = this->get_parameter("right_encoder_b").as_int();
    left_reverse_ = this->get_parameter("left_encoder_reverse").as_bool();
    right_reverse_ = this->get_parameter("right_encoder_reverse").as_bool();
    encoder_ppr_ = this->get_parameter("encoder_ppr").as_int();
    gear_ratio_ = this->get_parameter("gear_ratio").as_int();
    double pub_rate = this->get_parameter("publish_rate").as_double();
    double poll_rate = this->get_parameter("poll_rate").as_double();
    if (encoder_ppr_ <= 0 || gear_ratio_ <= 0 || pub_rate <= 0.0 || poll_rate <= 0.0) {
      throw std::invalid_argument("Encoder PPR, gear ratio, publish rate and poll rate must be positive");
    }

    // 2-edge quadrature decoding on Channel A (both rising and falling edges)
    ticks_per_rev_ = encoder_ppr_ * gear_ratio_ * 2;

    // Create encoder objects (polling-based, no alert callbacks)
    left_encoder_ = std::make_unique<Encoder>(chip_, left_a, left_b);
    right_encoder_ = std::make_unique<Encoder>(chip_, right_a, right_b);

    // Publisher
    wheel_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/wheel_states", 10);

    // Dedicated high-frequency polling thread (20 kHz default — reads GPIO pins)
    running_ = true;
    poll_thread_ = std::thread(&EncoderNode::pollLoop, this, poll_rate);

    // Publish timer (50 Hz default — publishes /wheel_states)
    auto pub_period = std::chrono::duration<double>(1.0 / pub_rate);
    pub_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(pub_period),
      std::bind(&EncoderNode::publishWheelStates, this));

    prev_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "EncoderNode started — L(A=%d B=%d rev=%d) R(A=%d B=%d rev=%d) PPR=%d GR=%d poll=%.0fHz",
      left_a, left_b, left_reverse_, right_a, right_b, right_reverse_, encoder_ppr_, gear_ratio_, poll_rate);
  }

  ~EncoderNode() override
  {
    running_ = false;
    if (poll_thread_.joinable()) {
      poll_thread_.join();
    }
  }

private:
  void pollLoop(double poll_rate)
  {
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / poll_rate));
    auto next_poll = std::chrono::steady_clock::now();
    while (running_.load()) {
      left_encoder_->poll();
      right_encoder_->poll();
      next_poll += period;
      std::this_thread::sleep_until(next_poll);
      if (std::chrono::steady_clock::now() > next_poll + period) {
        next_poll = std::chrono::steady_clock::now();
      }
    }
  }

  void publishWheelStates()
  {
    auto now = this->now();
    double dt = (now - prev_time_).seconds();
    prev_time_ = now;

    if (dt <= 0.0) return;

    int64_t left_ticks = left_encoder_->getTicks();
    int64_t right_ticks = right_encoder_->getTicks();

    if (left_reverse_) left_ticks = -left_ticks;
    if (right_reverse_) right_ticks = -right_ticks;

    // Debug: log raw ticks every second (~50 iterations at 50Hz)
    debug_counter_++;
    if (debug_counter_ % 50 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "[ENC] L_ticks=%ld  R_ticks=%ld", left_ticks, right_ticks);
    }

    // Compute raw velocities in rad/s
    double rads_per_tick = (2.0 * M_PI) / ticks_per_rev_;
    double raw_left_vel = (left_ticks - prev_left_ticks_) * rads_per_tick / dt;
    double raw_right_vel = (right_ticks - prev_right_ticks_) * rads_per_tick / dt;
    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;

    // Exponential moving average to smooth velocity
    constexpr double alpha = 0.3;
    left_vel_filtered_ = alpha * raw_left_vel + (1.0 - alpha) * left_vel_filtered_;
    right_vel_filtered_ = alpha * raw_right_vel + (1.0 - alpha) * right_vel_filtered_;
    double left_vel = left_vel_filtered_;
    double right_vel = right_vel_filtered_;

    // Compute positions in radians
    double left_pos = left_ticks * rads_per_tick;
    double right_pos = right_ticks * rads_per_tick;

    // Publish JointState
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = now;
    msg.name = {"left_wheel_joint", "right_wheel_joint"};
    msg.position = {left_pos, right_pos};
    msg.velocity = {left_vel, right_vel};
    wheel_pub_->publish(msg);
  }

  int chip_;
  bool left_reverse_{false};
  bool right_reverse_{false};
  int encoder_ppr_;
  int gear_ratio_;
  int ticks_per_rev_;

  std::unique_ptr<Encoder> left_encoder_;
  std::unique_ptr<Encoder> right_encoder_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;    // 50 Hz ROS publish

  std::atomic<bool> running_{false};
  std::thread poll_thread_;

  rclcpp::Time prev_time_;
  int64_t prev_left_ticks_{0};
  int64_t prev_right_ticks_{0};
  double left_vel_filtered_{0.0};
  double right_vel_filtered_{0.0};
  int debug_counter_{0};
};

// ============================================================================
// MotorDriverNode
// ============================================================================
class MotorDriverNode : public rclcpp::Node
{
public:
  MotorDriverNode(int chip_handle, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("motor_driver_node", options), chip_(chip_handle)
  {
    // Declare parameters (Smartelex 15D: PWM + DIR per motor)
    this->declare_parameter("left_pwm_pin", 18);
    this->declare_parameter("left_dir_pin", 23);
    this->declare_parameter("right_pwm_pin", 19);
    this->declare_parameter("right_dir_pin", 24);
    this->declare_parameter("left_motor_reverse", false);
    this->declare_parameter("right_motor_reverse", false);
    this->declare_parameter("left_motor_scale", 1.0);
    this->declare_parameter("right_motor_scale", 1.0);
    this->declare_parameter("wheel_radius", 0.035);
    this->declare_parameter("wheel_separation", 0.1702);
    this->declare_parameter("pid_kp", 1.0);
    this->declare_parameter("pid_ki", 0.2);
    this->declare_parameter("pid_kd", 0.05);
    this->declare_parameter("max_rpm", 200.0);
    this->declare_parameter("pid_rate", 50.0);
    this->declare_parameter("cmd_vel_timeout", 0.5);

    // Read parameters
    int left_pwm = this->get_parameter("left_pwm_pin").as_int();
    int left_dir = this->get_parameter("left_dir_pin").as_int();
    int right_pwm = this->get_parameter("right_pwm_pin").as_int();
    int right_dir = this->get_parameter("right_dir_pin").as_int();
    left_reverse_ = this->get_parameter("left_motor_reverse").as_bool();
    right_reverse_ = this->get_parameter("right_motor_reverse").as_bool();
    left_scale_ = this->get_parameter("left_motor_scale").as_double();
    right_scale_ = this->get_parameter("right_motor_scale").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    double kp = this->get_parameter("pid_kp").as_double();
    double ki = this->get_parameter("pid_ki").as_double();
    double kd = this->get_parameter("pid_kd").as_double();
    max_rpm_ = this->get_parameter("max_rpm").as_double();
    double rate = this->get_parameter("pid_rate").as_double();
    cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
    if (wheel_radius_ <= 0.0 || wheel_separation_ <= 0.0 || max_rpm_ <= 0.0 ||
        rate <= 0.0 || cmd_vel_timeout_ <= 0.0)
    {
      throw std::invalid_argument("Motor geometry, max RPM, PID rate and command timeout must be positive");
    }

    // Create Smartelex 15D motor drivers (PWM + DIR, 16 kHz)
    left_motor_ = std::make_unique<MotorDriver>(chip_, left_pwm, left_dir);
    right_motor_ = std::make_unique<MotorDriver>(chip_, right_pwm, right_dir);

    // Create PID controllers (output range: -30 to +30 duty cycle trim around feedforward)
    left_pid_ = std::make_unique<PIDController>(kp, ki, kd, 30.0, 20.0);
    right_pid_ = std::make_unique<PIDController>(kp, ki, kd, 30.0, 20.0);

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MotorDriverNode::cmdVelCallback, this, std::placeholders::_1));

    wheel_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/wheel_states", 10,
      std::bind(&MotorDriverNode::wheelStateCallback, this, std::placeholders::_1));

    // PID control loop timer
    auto period = std::chrono::duration<double>(1.0 / rate);
    pid_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MotorDriverNode::pidLoop, this));

    last_cmd_vel_time_ = this->now();
    prev_pid_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "MotorDriverNode [Smartelex 15D] started — L(PWM=%d DIR=%d rev=%d scale=%.2f) R(PWM=%d DIR=%d rev=%d scale=%.2f)",
      left_pwm, left_dir, left_reverse_, left_scale_, right_pwm, right_dir, right_reverse_, right_scale_);
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Convert (v, ω) to target RPM for each wheel
    double v = msg->linear.x;
    double omega = msg->angular.z;

    double v_left = v - omega * wheel_separation_ / 2.0;
    double v_right = v + omega * wheel_separation_ / 2.0;

    // Convert m/s to RPM:  RPM = v / (2π * r) * 60
    target_rpm_left_ = (v_left / (2.0 * M_PI * wheel_radius_)) * 60.0;
    target_rpm_right_ = (v_right / (2.0 * M_PI * wheel_radius_)) * 60.0;

    // Clamp to max RPM
    target_rpm_left_ = std::clamp(target_rpm_left_, -max_rpm_, max_rpm_);
    target_rpm_right_ = std::clamp(target_rpm_right_, -max_rpm_, max_rpm_);

    RCLCPP_INFO_ONCE(this->get_logger(),
      "[CMD] First cmd_vel received: v=%.3f ω=%.3f → target L=%.1f R=%.1f RPM",
      v, omega, target_rpm_left_, target_rpm_right_);

    last_cmd_vel_time_ = this->now();
  }

  void wheelStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->velocity.size() >= 2) {
      // velocity is in rad/s — convert to RPM
      current_rpm_left_ = msg->velocity[0] * 60.0 / (2.0 * M_PI);
      current_rpm_right_ = msg->velocity[1] * 60.0 / (2.0 * M_PI);
    }
  }

  void pidLoop()
  {
    auto now = this->now();
    double dt = (now - prev_pid_time_).seconds();
    prev_pid_time_ = now;

    if (dt <= 0.0) return;

    // Safety: stop motors if no cmd_vel received recently
    double time_since_cmd = (now - last_cmd_vel_time_).seconds();
    if (time_since_cmd > cmd_vel_timeout_) {
      left_motor_->stop();
      right_motor_->stop();
      left_pid_->reset();
      right_pid_->reset();
      target_rpm_left_ = 0.0;
      target_rpm_right_ = 0.0;
      return;
    }

    // Process Left Motor: Feedforward + Minimum PWM Deadband + PID correction
    if (std::abs(target_rpm_left_) < 0.01) {
      left_motor_->stop();
      left_pid_->reset();
    } else {
      double feedforward = (max_rpm_ > 0.0) ? (target_rpm_left_ / max_rpm_) * 100.0 : 0.0;
      double deadband = (target_rpm_left_ > 0.0) ? 12.0 : -12.0;
      double pid_corr = left_pid_->compute(target_rpm_left_, current_rpm_left_, dt);
      double left_output = std::clamp(feedforward + deadband + pid_corr, -100.0, 100.0) * left_scale_;
      if (left_reverse_) left_output = -left_output;
      left_motor_->setSpeed(left_output);
    }

    // Process Right Motor: Feedforward + Minimum PWM Deadband + PID correction
    if (std::abs(target_rpm_right_) < 0.01) {
      right_motor_->stop();
      right_pid_->reset();
    } else {
      double feedforward = (max_rpm_ > 0.0) ? (target_rpm_right_ / max_rpm_) * 100.0 : 0.0;
      double deadband = (target_rpm_right_ > 0.0) ? 12.0 : -12.0;
      double pid_corr = right_pid_->compute(target_rpm_right_, current_rpm_right_, dt);
      double right_output = std::clamp(feedforward + deadband + pid_corr, -100.0, 100.0) * right_scale_;
      if (right_reverse_) right_output = -right_output;
      right_motor_->setSpeed(right_output);
    }

    // Debug: log PID output every second
    pid_debug_counter_++;
    if (pid_debug_counter_ % 50 == 0 && (target_rpm_left_ != 0.0 || target_rpm_right_ != 0.0)) {
      RCLCPP_INFO(this->get_logger(),
        "[PID] target L=%.1f R=%.1f | actual L=%.1f R=%.1f",
        target_rpm_left_, target_rpm_right_,
        current_rpm_left_, current_rpm_right_);
    }
  }

  int chip_;
  bool left_reverse_{false};
  bool right_reverse_{false};
  double left_scale_{1.0};
  double right_scale_{1.0};
  double wheel_radius_;
  double wheel_separation_;
  double max_rpm_;
  double cmd_vel_timeout_;

  std::unique_ptr<MotorDriver> left_motor_;
  std::unique_ptr<MotorDriver> right_motor_;
  std::unique_ptr<PIDController> left_pid_;
  std::unique_ptr<PIDController> right_pid_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_state_sub_;
  rclcpp::TimerBase::SharedPtr pid_timer_;

  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time prev_pid_time_;

  double target_rpm_left_{0.0};
  double target_rpm_right_{0.0};
  double current_rpm_left_{0.0};
  double current_rpm_right_{0.0};
  int pid_debug_counter_{0};
};

// ============================================================================
// OdometryNode
// ============================================================================
class OdometryNode : public rclcpp::Node
{
public:
  OdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("odometry_node", options)
  {
    // Declare parameters
    this->declare_parameter("wheel_radius", 0.033);
    this->declare_parameter("wheel_separation", 0.17);
    this->declare_parameter("encoder_ppr", 11);
    this->declare_parameter("gear_ratio", 30);
    this->declare_parameter("odom_frame", std::string("odom"));
    this->declare_parameter("base_frame", std::string("base_footprint"));
    this->declare_parameter("publish_tf", true);  // Set false when EKF handles odom→base_footprint TF

    // Read parameters
    double radius = this->get_parameter("wheel_radius").as_double();
    double separation = this->get_parameter("wheel_separation").as_double();
    int ppr = this->get_parameter("encoder_ppr").as_int();
    int gear = this->get_parameter("gear_ratio").as_int();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    if (radius <= 0.0 || separation <= 0.0 || ppr <= 0 || gear <= 0) {
      throw std::invalid_argument("Odometry geometry, encoder PPR and gear ratio must be positive");
    }

    // 2-edge quadrature decoding on Channel A (both rising and falling edges)
    int ticks_per_rev = ppr * gear * 2;
    odom_ = std::make_unique<Odometry>(radius, separation, ticks_per_rev);

    // Subscriber
    wheel_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/wheel_states", 10,
      std::bind(&OdometryNode::wheelStateCallback, this, std::placeholders::_1));

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    prev_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "OdometryNode started — radius=%.3f sep=%.3f ticks/rev=%d publish_tf=%s",
      radius, separation, ticks_per_rev, publish_tf_ ? "true" : "false");
  }

private:
  void wheelStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < 2 || msg->velocity.size() < 2) {
      return;
    }

    auto now = this->now();
    double dt = (now - prev_time_).seconds();
    prev_time_ = now;

    if (dt <= 0.0) return;

    // Convert position (radians) back to ticks for the Odometry class
    double rads_per_tick = (2.0 * M_PI) /
      (this->get_parameter("encoder_ppr").as_int() *
       this->get_parameter("gear_ratio").as_int() * 2);
    int64_t left_ticks = static_cast<int64_t>(std::round(msg->position[0] / rads_per_tick));
    int64_t right_ticks = static_cast<int64_t>(std::round(msg->position[1] / rads_per_tick));

    // Update odometry
    odom_->update(left_ticks, right_ticks, dt);

    // Build quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_->getTheta());

    // --- Publish TF: odom → base_link (only if EKF is NOT handling it) ---
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = now;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = odom_->getX();
      tf.transform.translation.y = odom_->getY();
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }

    // --- Publish nav_msgs/Odometry ---
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Pose
    odom_msg.pose.pose.position.x = odom_->getX();
    odom_msg.pose.pose.position.y = odom_->getY();
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Pose covariance (6x6 row-major: x, y, z, roll, pitch, yaw)
    // Non-zero diagonal entries tell AMCL/EKF how much to trust odometry
    odom_msg.pose.covariance[0]  = 0.01;   // x variance
    odom_msg.pose.covariance[7]  = 0.01;   // y variance
    odom_msg.pose.covariance[14] = 1e6;    // z (unused in 2D, set high)
    odom_msg.pose.covariance[21] = 1e6;    // roll (unused in 2D)
    odom_msg.pose.covariance[28] = 1e6;    // pitch (unused in 2D)
    odom_msg.pose.covariance[35] = 0.03;   // yaw variance

    // Twist (in base_link frame)
    odom_msg.twist.twist.linear.x = odom_->getLinearVel();
    odom_msg.twist.twist.angular.z = odom_->getAngularVel();

    // Twist covariance
    odom_msg.twist.covariance[0]  = 0.01;  // vx variance
    odom_msg.twist.covariance[7]  = 0.01;  // vy variance
    odom_msg.twist.covariance[14] = 1e6;   // vz (unused)
    odom_msg.twist.covariance[21] = 1e6;   // vroll (unused)
    odom_msg.twist.covariance[28] = 1e6;   // vpitch (unused)
    odom_msg.twist.covariance[35] = 0.03;  // vyaw variance

    odom_pub_->publish(odom_msg);

    // --- Publish joint_states ---
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = now;
    joint_msg.name = {"left_wheel_joint", "right_wheel_joint"};
    joint_msg.position = {msg->position[0], msg->position[1]};
    joint_msg.velocity = {msg->velocity[0], msg->velocity[1]};
    joint_pub_->publish(joint_msg);
  }

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};

  std::unique_ptr<Odometry> odom_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time prev_time_;
};

}  // namespace diff_drive_robot


// ============================================================================
// main — single process, three nodes, shared GPIO chip handle
// ============================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Try to open GPIO chip — Pi 5 header pins are on gpiochip4 (pinctrl-rp1).
  // gpiochip0 is an internal SoC GPIO that does NOT control the 40-pin header.
  int chip = -1;
  for (int chip_num : {4, 0}) {
    chip = lgGpiochipOpen(chip_num);
    if (chip >= 0) {
      RCLCPP_INFO(rclcpp::get_logger("main"),
        "GPIO chip %d opened (handle=%d)", chip_num, chip);
      break;
    }
    RCLCPP_WARN(rclcpp::get_logger("main"),
      "Could not open gpiochip%d (error %d), trying next...", chip_num, chip);
  }

  if (chip < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("main"),
      "Failed to open any GPIO chip. "
      "Are you running as root or in the gpio group?");
    return 1;
  }

  int exit_code = 0;
  try {
    auto encoder_node = std::make_shared<diff_drive_robot::EncoderNode>(chip);
    auto motor_node = std::make_shared<diff_drive_robot::MotorDriverNode>(chip);
    auto odom_node = std::make_shared<diff_drive_robot::OdometryNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(encoder_node);
    executor.add_node(motor_node);
    executor.add_node(odom_node);

    RCLCPP_INFO(rclcpp::get_logger("main"),
      "All nodes ready. Spinning...");

    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("main"),
      "Exception: %s", e.what());
    exit_code = 1;
  }

  lgGpiochipClose(chip);
  rclcpp::shutdown();
  return exit_code;
}
