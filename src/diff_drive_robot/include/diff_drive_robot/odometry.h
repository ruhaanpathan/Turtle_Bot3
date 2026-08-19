#ifndef DIFF_DRIVE_ROBOT__ODOMETRY_H_
#define DIFF_DRIVE_ROBOT__ODOMETRY_H_

#include <cstdint>

namespace diff_drive_robot
{

/**
 * @brief Differential drive odometry from encoder ticks.
 *
 * Converts cumulative left/right encoder ticks into (x, y, θ) pose
 * and linear/angular velocities using standard diff-drive kinematics.
 */
class Odometry
{
public:
  /**
   * @brief Construct an Odometry calculator.
   * @param wheel_radius      Wheel radius in meters.
   * @param wheel_separation  Distance between wheel centers in meters.
   * @param ticks_per_rev     Total encoder ticks per output shaft revolution
   *                          (encoder_ppr × gear_ratio).
   */
  Odometry(double wheel_radius, double wheel_separation, int ticks_per_rev);

  /**
   * @brief Update odometry from current encoder tick counts.
   * @param left_ticks   Cumulative left encoder ticks.
   * @param right_ticks  Cumulative right encoder ticks.
   * @param dt           Time elapsed since last update (seconds).
   */
  void update(int64_t left_ticks, int64_t right_ticks, double dt);

  /**
   * @brief Reset pose to origin (0, 0, 0).
   */
  void reset();

  // --- Getters ---
  double getX() const { return x_; }
  double getY() const { return y_; }
  double getTheta() const { return theta_; }
  double getLinearVel() const { return linear_vel_; }
  double getAngularVel() const { return angular_vel_; }

  /** @brief Get left wheel position in radians. */
  double getLeftWheelPos() const;

  /** @brief Get right wheel position in radians. */
  double getRightWheelPos() const;

private:
  double wheel_radius_;
  double wheel_separation_;
  int ticks_per_rev_;

  // Pose
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};

  // Velocities
  double linear_vel_{0.0};
  double angular_vel_{0.0};

  // Previous tick counts
  int64_t prev_left_{0};
  int64_t prev_right_{0};
  bool initialized_{false};

  // Cumulative ticks for wheel position
  int64_t total_left_{0};
  int64_t total_right_{0};
};

}  // namespace diff_drive_robot

#endif  // DIFF_DRIVE_ROBOT__ODOMETRY_H_
