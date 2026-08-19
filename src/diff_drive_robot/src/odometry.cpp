#include "diff_drive_robot/odometry.h"

#include <cmath>

namespace diff_drive_robot
{

Odometry::Odometry(double wheel_radius, double wheel_separation,
                   int ticks_per_rev)
: wheel_radius_(wheel_radius),
  wheel_separation_(wheel_separation),
  ticks_per_rev_(ticks_per_rev)
{
}

void Odometry::update(int64_t left_ticks, int64_t right_ticks, double dt)
{
  if (!initialized_) {
    prev_left_ = left_ticks;
    prev_right_ = right_ticks;
    initialized_ = true;
    return;
  }

  if (dt <= 0.0) {
    return;
  }

  // Tick deltas since last update
  int64_t delta_left = left_ticks - prev_left_;
  int64_t delta_right = right_ticks - prev_right_;
  prev_left_ = left_ticks;
  prev_right_ = right_ticks;

  // Track cumulative ticks for wheel position
  total_left_ += delta_left;
  total_right_ += delta_right;

  // Convert tick deltas to wheel rotation angles (radians)
  double rads_per_tick = (2.0 * M_PI) / ticks_per_rev_;
  double dphi_l = delta_left * rads_per_tick;
  double dphi_r = delta_right * rads_per_tick;

  // Body displacement twist from turtlelib::DiffDrive forward kinematics
  // w = (r / track_width) * (dphi_r - dphi_l)
  // v_x = (r / 2.0) * (dphi_r + dphi_l)
  double delta_theta = (wheel_radius_ / wheel_separation_) * (dphi_r - dphi_l);
  double delta_x_body = (wheel_radius_ / 2.0) * (dphi_r + dphi_l);

  // Integrate body twist along curved arc (turtlelib SE(2) integration)
  double dx = 0.0;
  double dy = 0.0;
  if (std::abs(delta_theta) > 1e-6) {
    dx = (delta_x_body / delta_theta) * std::sin(delta_theta);
    dy = (delta_x_body / delta_theta) * (1.0 - std::cos(delta_theta));
  } else {
    dx = delta_x_body;
    dy = 0.0;
  }

  // Transform body displacement to world frame using current heading theta_
  x_ += dx * std::cos(theta_) - dy * std::sin(theta_);
  y_ += dx * std::sin(theta_) + dy * std::cos(theta_);
  theta_ += delta_theta;

  // Normalize theta to [-pi, pi]
  theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

  // Compute velocities (m/s and rad/s)
  linear_vel_ = delta_x_body / dt;
  angular_vel_ = delta_theta / dt;
}

void Odometry::reset()
{
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
  linear_vel_ = 0.0;
  angular_vel_ = 0.0;
  prev_left_ = 0;
  prev_right_ = 0;
  total_left_ = 0;
  total_right_ = 0;
  initialized_ = false;
}

double Odometry::getLeftWheelPos() const
{
  return (2.0 * M_PI * total_left_) / ticks_per_rev_;
}

double Odometry::getRightWheelPos() const
{
  return (2.0 * M_PI * total_right_) / ticks_per_rev_;
}

}  // namespace diff_drive_robot
