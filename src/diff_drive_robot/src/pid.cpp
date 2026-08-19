#include "diff_drive_robot/pid.h"

#include <algorithm>
#include <cmath>

namespace diff_drive_robot
{

PIDController::PIDController(double kp, double ki, double kd,
                             double max_output, double max_integral)
: kp_(kp), ki_(ki), kd_(kd),
  max_output_(max_output), max_integral_(max_integral)
{
}

double PIDController::compute(double setpoint, double measurement, double dt)
{
  if (dt <= 0.0) {
    return 0.0;
  }

  // If target setpoint is zero (or near zero), reset integral and return zero speed
  if (std::abs(setpoint) < 0.01) {
    reset();
    return 0.0;
  }

  double error = setpoint - measurement;

  // Integral with the caller-provided anti-windup clamp.
  integral_ += error * dt;
  integral_ = std::clamp(integral_, -max_integral_, max_integral_);

  // Derivative on measurement (avoids massive kick when setpoint jumps)
  double derivative = -(measurement - prev_measurement_) / dt;
  prev_measurement_ = measurement;

  // PID output
  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // Clamp output
  output = std::clamp(output, -max_output_, max_output_);

  return output;
}

void PIDController::reset()
{
  integral_ = 0.0;
  prev_measurement_ = 0.0;
}

void PIDController::setGains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  reset();
}

}  // namespace diff_drive_robot
