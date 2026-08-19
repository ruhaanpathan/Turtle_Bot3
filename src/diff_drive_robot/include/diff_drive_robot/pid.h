#ifndef DIFF_DRIVE_ROBOT__PID_H_
#define DIFF_DRIVE_ROBOT__PID_H_

namespace diff_drive_robot
{

/**
 * @brief Discrete PID controller with anti-windup clamping.
 *
 * Computes control output from setpoint error. Integral term is
 * clamped to prevent windup. Output is clamped to ±max_output.
 */
class PIDController
{
public:
  /**
   * @brief Construct a PID controller.
   * @param kp            Proportional gain.
   * @param ki            Integral gain.
   * @param kd            Derivative gain.
   * @param max_output    Output clamp magnitude (e.g. 100.0 for duty cycle).
   * @param max_integral  Integral term clamp to prevent windup.
   */
  PIDController(double kp, double ki, double kd,
                double max_output = 100.0, double max_integral = 50.0);

  /**
   * @brief Compute PID output for one timestep.
   * @param setpoint     Desired value (e.g. target RPM).
   * @param measurement  Current measured value (e.g. actual RPM).
   * @param dt           Time elapsed since last call (seconds). Must be > 0.
   * @return Control output clamped to ±max_output.
   */
  double compute(double setpoint, double measurement, double dt);

  /**
   * @brief Reset integral accumulator and previous error.
   */
  void reset();

  /**
   * @brief Update PID gains at runtime.
   */
  void setGains(double kp, double ki, double kd);

private:
  double kp_, ki_, kd_;
  double max_output_;
  double max_integral_;
  double integral_{0.0};
  double prev_measurement_{0.0};
};

}  // namespace diff_drive_robot

#endif  // DIFF_DRIVE_ROBOT__PID_H_
