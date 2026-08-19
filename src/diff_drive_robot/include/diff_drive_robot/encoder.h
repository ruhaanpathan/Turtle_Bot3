#ifndef DIFF_DRIVE_ROBOT__ENCODER_H_
#define DIFF_DRIVE_ROBOT__ENCODER_H_

#include <atomic>
#include <cstdint>
#include <lgpio.h>

namespace diff_drive_robot
{

/**
 * @brief Quadrature encoder reader using high-frequency GPIO polling.
 *
 * Polls channel A and B at each call to poll(), detects edges by comparing
 * with previous state, and increments/decrements the tick counter.
 *
 * This is more reliable than lgpio alert callbacks on Pi 5, which drop
 * a large percentage of edges at high RPM.
 */
class Encoder
{
public:
  /**
   * @brief Construct an Encoder.
   * @param chip_handle  lgpio chip handle from lgGpiochipOpen().
   * @param pin_a        BCM GPIO pin for encoder channel A.
   * @param pin_b        BCM GPIO pin for encoder channel B.
   */
  Encoder(int chip_handle, int pin_a, int pin_b);

  ~Encoder();

  /**
   * @brief Poll the GPIO pins and update tick count.
   *        Call this from a high-frequency timer (1-5 kHz).
   */
  void poll();

  /**
   * @brief Get the current cumulative tick count (thread-safe).
   * @return Signed tick count. Positive = forward.
   */
  int64_t getTicks() const;

  /**
   * @brief Reset tick count to zero.
   */
  void reset();

private:
  int chip_;
  int pin_a_;
  int pin_b_;
  std::atomic<int64_t> ticks_{0};
  int prev_a_{-1};  // Previous state of channel A (-1 = uninitialized)
  int prev_b_{-1};  // Previous state of channel B
};

}  // namespace diff_drive_robot

#endif  // DIFF_DRIVE_ROBOT__ENCODER_H_
