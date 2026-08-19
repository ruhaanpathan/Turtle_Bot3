#ifndef DIFF_DRIVE_ROBOT_ML_H
#define DIFF_DRIVE_ROBOT_ML_H

#include "diff_drive_robot/ekf.h"
#include <vector>
#include <armadillo>

namespace turtlelib
{
    struct RangeID
    {
        double range;
        double angle;
        int cluster;

        RangeID();
        RangeID(double range, double angle);
    };

    struct Clusters
    {
        std::vector<RangeID> ranges;
        int n_clusters;
    };

    struct ClustersCentroids
    {
        std::vector<std::vector<arma::vec>> points;
        std::vector<Vector2D> centroids;
    };

    Clusters clustering(
        const std::vector<double> & range_data,
        double angle_min,
        double angle_increment,
        double dist_threshold);
    std::vector<Vector2D> centroid_finder(Clusters cluster);
    ClustersCentroids shift_points(Clusters cluster, std::vector<Vector2D> centroids);
    std::vector<Circle> circle_detection(ClustersCentroids clusters);
    std::vector<bool> classification(std::vector<Circle> detected_circles);
}

#endif
