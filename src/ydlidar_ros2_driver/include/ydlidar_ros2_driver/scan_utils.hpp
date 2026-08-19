// Copyright 2026 YDLIDAR ROS 2 driver contributors
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

#ifndef YDLIDAR_ROS2_DRIVER__SCAN_UTILS_HPP_
#define YDLIDAR_ROS2_DRIVER__SCAN_UTILS_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace ydlidar_ros2_driver
{

constexpr std::size_t kMaximumScanBins = 1000000U;

inline bool calculate_scan_bin_count(
  const float angle_min, const float angle_max, const float angle_increment,
  std::size_t & bin_count)
{
  if (!std::isfinite(angle_min) || !std::isfinite(angle_max) ||
    !std::isfinite(angle_increment) || angle_increment <= 0.0F || angle_max < angle_min)
  {
    return false;
  }

  const double intervals =
    (static_cast<double>(angle_max) - static_cast<double>(angle_min)) /
    static_cast<double>(angle_increment);
  if (!std::isfinite(intervals) || intervals < 0.0 ||
    intervals > static_cast<double>(kMaximumScanBins - 1U))
  {
    return false;
  }

  // LaserScan bins are centered at angle_min + index * angle_increment.
  // Use only centers that do not exceed angle_max. Rounding could otherwise
  // advertise a final bin outside a cropped, non-integral field of view.
  bin_count = static_cast<std::size_t>(std::floor(intervals)) + 1U;
  return bin_count > 0U && bin_count <= kMaximumScanBins;
}

inline float scan_angle_max(
  const float angle_min, const float angle_increment, const std::size_t bin_count)
{
  if (bin_count == 0U) {
    return angle_min;
  }
  return angle_min +
         static_cast<float>(bin_count - 1U) * angle_increment;
}

inline bool scan_bin_index(
  const float angle, const float angle_min, const float angle_increment,
  const std::size_t bin_count, std::size_t & index)
{
  if (!std::isfinite(angle) || !std::isfinite(angle_min) ||
    !std::isfinite(angle_increment) || angle_increment <= 0.0F || bin_count == 0U)
  {
    return false;
  }

  const double raw_index =
    (static_cast<double>(angle) - static_cast<double>(angle_min)) /
    static_cast<double>(angle_increment);
  if (!std::isfinite(raw_index) || raw_index < -0.5 ||
    raw_index > static_cast<double>(bin_count) - 0.5)
  {
    return false;
  }

  const int64_t rounded_index = std::llround(raw_index);
  if (rounded_index < 0 ||
    rounded_index >= static_cast<int64_t>(bin_count))
  {
    return false;
  }

  index = static_cast<std::size_t>(rounded_index);
  return true;
}

inline bool valid_scan_range(const float range, const float range_min, const float range_max)
{
  return std::isfinite(range) && range >= range_min && range <= range_max;
}

inline float invalid_scan_range(const bool use_infinity)
{
  return use_infinity ? std::numeric_limits<float>::infinity() : 0.0F;
}

}  // namespace ydlidar_ros2_driver

#endif  // YDLIDAR_ROS2_DRIVER__SCAN_UTILS_HPP_
