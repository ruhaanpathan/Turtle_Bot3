#include "diff_drive_robot/motor_driver.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace diff_drive_robot
{

MotorDriver::MotorDriver(int chip_handle, int pwm_pin, int dir_pin,
                         int pwm_freq)
: chip_(chip_handle),
  pwm_pin_(pwm_pin),
  dir_pin_(dir_pin),
  pwm_freq_(pwm_freq)
{
  int rc;

  // Claim PWM pin as output, initially LOW
  rc = lgGpioClaimOutput(chip_, 0, pwm_pin_, 0);
  if (rc < 0) {
    throw std::runtime_error("Failed to claim PWM pin " +
                             std::to_string(pwm_pin_));
  }

  // Claim DIR pin as output, initially LOW
  rc = lgGpioClaimOutput(chip_, 0, dir_pin_, 0);
  if (rc < 0) {
    throw std::runtime_error("Failed to claim DIR pin " +
                             std::to_string(dir_pin_));
  }
}

MotorDriver::~MotorDriver()
{
  stop();
  // lgpio automatically frees GPIOs when chip is closed
}

void MotorDriver::setSpeed(double duty_cycle)
{
  // Clamp to [-100, 100]
  duty_cycle = std::clamp(duty_cycle, -100.0, 100.0);

  // Determine direction from sign
  if (duty_cycle > 0.0) {
    // Forward: DIR=HIGH
    lgGpioWrite(chip_, dir_pin_, 1);
  } else if (duty_cycle < 0.0) {
    // Reverse: DIR=LOW
    lgGpioWrite(chip_, dir_pin_, 0);
  } else {
    // Zero speed — stop
    stop();
    return;
  }

  // Set PWM with absolute duty cycle
  double abs_duty = std::abs(duty_cycle);
  lgTxPwm(chip_, pwm_pin_, pwm_freq_, abs_duty, 0, 0);
}

void MotorDriver::stop()
{
  // Stop: DIR=LOW, PWM off
  lgGpioWrite(chip_, dir_pin_, 0);
  lgTxPwm(chip_, pwm_pin_, pwm_freq_, 0, 0, 0);
}

}  // namespace diff_drive_robot
