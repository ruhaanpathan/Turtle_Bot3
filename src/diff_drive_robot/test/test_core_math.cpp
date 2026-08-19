#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "diff_drive_robot/diff_drive_lib.h"
#include "diff_drive_robot/ekf.h"
#include "diff_drive_robot/ml.h"
#include "diff_drive_robot/rigid2d.h"

namespace
{
constexpr double kTolerance = 1.0e-9;

TEST(Rigid2D, IntegratesStraightTwist)
{
  const turtlelib::Transform2D transform =
    turtlelib::integrate_twist({0.0, {1.0, 0.2}});

  EXPECT_NEAR(transform.translation().x, 1.0, kTolerance);
  EXPECT_NEAR(transform.translation().y, 0.2, kTolerance);
  EXPECT_NEAR(transform.rotation(), 0.0, kTolerance);
}

TEST(Rigid2D, IntegratesArcTwist)
{
  const turtlelib::Transform2D transform =
    turtlelib::integrate_twist({turtlelib::PI / 2.0, {1.0, 0.0}});
  const double radius = 2.0 / turtlelib::PI;

  EXPECT_NEAR(transform.translation().x, radius, kTolerance);
  EXPECT_NEAR(transform.translation().y, radius, kTolerance);
  EXPECT_NEAR(transform.rotation(), turtlelib::PI / 2.0, kTolerance);
}

TEST(DiffDrive, AppliesBodyMotionAtPreviousHeading)
{
  turtlelib::DiffDrive robot(
    0.17, 0.033, turtlelib::Config{0.0, 0.0, turtlelib::PI / 2.0});

  robot.forward_kin(1.0, 1.0);

  EXPECT_NEAR(robot.q.x, 0.0, kTolerance);
  EXPECT_NEAR(robot.q.y, 0.033, kTolerance);
  EXPECT_NEAR(robot.q.theta, turtlelib::PI / 2.0, kTolerance);
}

TEST(Clustering, UsesAngleMinAndSplitsAcrossInvalidGap)
{
  const auto clusters = turtlelib::clustering(
    {1.0, 1.0, std::numeric_limits<double>::infinity(), 1.0, 1.0},
    -1.0, 0.1, 0.2);

  ASSERT_EQ(clusters.ranges.size(), 5U);
  EXPECT_NEAR(clusters.ranges.front().angle, -1.0, kTolerance);
  EXPECT_EQ(clusters.ranges[0].cluster, 0);
  EXPECT_EQ(clusters.ranges[1].cluster, 0);
  EXPECT_EQ(clusters.ranges[2].cluster, -1);
  EXPECT_EQ(clusters.ranges[3].cluster, 1);
  EXPECT_EQ(clusters.ranges[4].cluster, 1);
  EXPECT_EQ(clusters.n_clusters, 2);
}

TEST(Clustering, SupportsClustersLargerThanLegacyFixedBuffer)
{
  constexpr std::size_t point_count = 500;
  const double angle_increment = 0.001;
  const auto clusters = turtlelib::clustering(
    std::vector<double>(point_count, 1.0), 0.0, angle_increment, 0.01);
  const auto centroids = turtlelib::centroid_finder(clusters);
  const auto shifted = turtlelib::shift_points(clusters, centroids);

  ASSERT_EQ(clusters.n_clusters, 1);
  ASSERT_EQ(shifted.points.size(), 1U);
  EXPECT_EQ(shifted.points.front().at(0).n_elem, point_count);
  EXPECT_EQ(shifted.points.front().at(1).n_elem, point_count);
}

TEST(EkfSlam, ComposesOdometryIncrementInCorrectedMapFrame)
{
  turtlelib::EKF ekf({0.0, 0.0, 0.0}, 1, 0.01, 0.01);
  ekf.zeta_est(0) = turtlelib::PI / 2.0;

  ekf.prediction({1.0, 0.0, 0.0});

  EXPECT_NEAR(ekf.zeta_est(1), 0.0, kTolerance);
  EXPECT_NEAR(ekf.zeta_est(2), 1.0, kTolerance);
  EXPECT_NEAR(ekf.zeta_est(0), turtlelib::PI / 2.0, kTolerance);
}

TEST(EkfSlam, WrapsBearingInnovationAtPiBoundary)
{
  turtlelib::EKF ekf({0.0, 0.0, 0.0}, 1, 0.01, 0.01);
  const double initial_angle = turtlelib::PI - 0.01;
  ekf.initialization(0, std::cos(initial_angle), std::sin(initial_angle));
  ekf.izd.at(0) = true;

  const double observed_angle = -turtlelib::PI + 0.01;
  const turtlelib::Circle observation{
    std::cos(observed_angle), std::sin(observed_angle), 0.05};

  const double distance = ekf.mah_distance(observation, 0);
  EXPECT_TRUE(std::isfinite(distance));
  EXPECT_LT(distance, 1.0);
}
}  // namespace
