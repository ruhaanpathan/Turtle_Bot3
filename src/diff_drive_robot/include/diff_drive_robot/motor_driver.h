#ifndef DIFF_DRIVE_ROBOT__MOTOR_DRIVER_H_
#define DIFF_DRIVE_ROBOT__MOTOR_DRIVER_H_

#include <lgpio.h>

namespace diff_drive_robot
{

/**
 * @brief Hardware abstraction for one motor via Smartelex 15D PWM+DIR driver.
 *
 * Controls direction (DIR) and speed (PWM) using the lgpio C library.
 * Each MotorDriver instance drives one motor channel (left or right).
 *
 * The Smartelex 15D uses only 2 control pins per motor:
 *   - PWM pin: duty cycle 0–100% controls speed
 *   - DIR pin: HIGH = forward, LOW = reverse
 */
class MotorDriver
{
public:
  /**
   * @brief Construct a MotorDriver for Smartelex 15D.
   * @param chip_handle  lgpio chip handle from lgGpiochipOpen().
   * @param pwm_pin      BCM GPIO pin for PWM speed control.
   * @param dir_pin      BCM GPIO pin for DIR direction control.
   * @param pwm_freq     PWM frequency in Hz (default 1000 for Smartelex 15D).
   */
  MotorDriver(int chip_handle, int pwm_pin, int dir_pin,
              int pwm_freq = 1000);

  ~MotorDriver();

  /**
   * @brief Set motor speed and direction.
   * @param duty_cycle  Value from -100.0 (full reverse) to +100.0 (full forward).
   *                    Sign determines direction via DIR pin. Clamped internally.
   */
  void setSpeed(double duty_cycle);

  /**
   * @brief Stop the motor (DIR=LOW, PWM=0).
   */
  void stop();

private:
  int chip_;
  int pwm_pin_;
  int dir_pin_;
  int pwm_freq_;
};

}  // namespace diff_drive_robot

#endif  // DIFF_DRIVE_ROBOT__MOTOR_DRIVER_H_
