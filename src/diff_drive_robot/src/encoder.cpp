#include "diff_drive_robot/encoder.h"

#include <stdexcept>

namespace diff_drive_robot
{

Encoder::Encoder(int chip_handle, int pin_a, int pin_b)
: chip_(chip_handle), pin_a_(pin_a), pin_b_(pin_b)
{
  int rc;

  // Claim both channels as inputs with internal pull-up resistors
  rc = lgGpioClaimInput(chip_, LG_SET_PULL_UP, pin_a_);
  if (rc < 0) {
    throw std::runtime_error("Failed to claim encoder pin A " +
                             std::to_string(pin_a_));
  }

  rc = lgGpioClaimInput(chip_, LG_SET_PULL_UP, pin_b_);
  if (rc < 0) {
    throw std::runtime_error("Failed to claim encoder pin B " +
                             std::to_string(pin_b_));
  }

  // Read initial state
  prev_a_ = lgGpioRead(chip_, pin_a_);
  prev_b_ = lgGpioRead(chip_, pin_b_);
}

Encoder::~Encoder()
{
  // lgpio frees GPIOs when chip is closed
}

void Encoder::poll()
{
  int a = lgGpioRead(chip_, pin_a_);
  int b = lgGpioRead(chip_, pin_b_);

  if (a < 0 || b < 0) return;  // Read error

  // Detect edges on channel A
  if (a != prev_a_) {
    // Edge detected on A — use B to determine direction
    if (a == b) {
      ticks_.fetch_add(1, std::memory_order_relaxed);
    } else {
      ticks_.fetch_sub(1, std::memory_order_relaxed);
    }
  }

  prev_a_ = a;
  prev_b_ = b;
}

int64_t Encoder::getTicks() const
{
  return ticks_.load(std::memory_order_relaxed);
}

void Encoder::reset()
{
  ticks_.store(0, std::memory_order_relaxed);
}

}  // namespace diff_drive_robot
