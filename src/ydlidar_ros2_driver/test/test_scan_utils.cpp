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

#include <cmath>
#include <cstddef>
#include <limits>

#include "gtest/gtest.h"
#include "ydlidar_ros2_driver/scan_utils.hpp"

TEST(ScanUtils, CalculatesInclusiveBinCount)
{
  std::size_t count = 0U;
  ASSERT_TRUE(ydlidar_ros2_driver::calculate_scan_bin_count(-1.0F, 1.0F, 0.01F, count));
  EXPECT_EQ(count, 201U);
}

TEST(ScanUtils, KeepsFractionalFieldOfViewInsideAdvertisedMaximum)
{
  std::size_t count = 0U;
  ASSERT_TRUE(ydlidar_ros2_driver::calculate_scan_bin_count(0.0F, 1.0F, 0.3F, count));
  ASSERT_EQ(count, 4U);
  const float published_max = ydlidar_ros2_driver::scan_angle_max(0.0F, 0.3F, count);
  EXPECT_FLOAT_EQ(published_max, 0.9F);
  EXPECT_LE(published_max, 1.0F);
}

TEST(ScanUtils, RejectsInvalidGeometry)
{
  std::size_t count = 0U;
  EXPECT_FALSE(ydlidar_ros2_driver::calculate_scan_bin_count(1.0F, -1.0F, 0.01F, count));
  EXPECT_FALSE(ydlidar_ros2_driver::calculate_scan_bin_count(-1.0F, 1.0F, 0.0F, count));
  EXPECT_FALSE(ydlidar_ros2_driver::calculate_scan_bin_count(
      -1.0F, 1.0F, std::nanf(""), count));
}

TEST(ScanUtils, RoundsSamplesToNearestBin)
{
  std::size_t index = 0U;
  ASSERT_TRUE(ydlidar_ros2_driver::scan_bin_index(0.049F, 0.0F, 0.1F, 5U, index));
  EXPECT_EQ(index, 0U);
  ASSERT_TRUE(ydlidar_ros2_driver::scan_bin_index(0.051F, 0.0F, 0.1F, 5U, index));
  EXPECT_EQ(index, 1U);
  EXPECT_FALSE(ydlidar_ros2_driver::scan_bin_index(0.5F, 0.0F, 0.1F, 5U, index));
}

TEST(ScanUtils, RejectsNonFiniteAndOutOfRangeReturns)
{
  EXPECT_TRUE(ydlidar_ros2_driver::valid_scan_range(0.12F, 0.12F, 8.0F));
  EXPECT_TRUE(ydlidar_ros2_driver::valid_scan_range(8.0F, 0.12F, 8.0F));
  EXPECT_FALSE(ydlidar_ros2_driver::valid_scan_range(0.0F, 0.12F, 8.0F));
  EXPECT_FALSE(ydlidar_ros2_driver::valid_scan_range(8.1F, 0.12F, 8.0F));
  EXPECT_FALSE(ydlidar_ros2_driver::valid_scan_range(
      std::numeric_limits<float>::infinity(), 0.12F, 8.0F));
}

TEST(ScanUtils, HonorsInvalidRangePolicy)
{
  EXPECT_FLOAT_EQ(ydlidar_ros2_driver::invalid_scan_range(false), 0.0F);
  EXPECT_TRUE(std::isinf(ydlidar_ros2_driver::invalid_scan_range(true)));
}
