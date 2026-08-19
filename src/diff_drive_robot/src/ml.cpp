#include "diff_drive_robot/ml.h"
#include <cmath>
#include <iostream>
#include <cfloat>

namespace turtlelib
{
    RangeID::RangeID()
     : range{0.0}, angle{0.0}, cluster{-1}
    {}

    RangeID::RangeID(double range, double angle)
     : range{range}, angle{angle}, cluster{-1}
    {}

    Clusters clustering(
        const std::vector<double> & range_data,
        double angle_min,
        double angle_increment,
        double dist_threshold)
    {
        const int n = static_cast<int>(range_data.size());
        if (n == 0) return Clusters{{}, 0};

        int cluster_id{-1};
        Clusters lidar{};
        lidar.ranges.reserve(range_data.size());

        bool previous_valid = false;
        double previous_range = 0.0;
        for(int i{0}; i < n; i++)
        {
            RangeID range_i{};
            range_i.angle = angle_min + static_cast<double>(i)*angle_increment;
            const double current_range = range_data.at(i);
            const bool current_valid = std::isfinite(current_range) && current_range > 0.0;

            if (current_valid)
            {
                range_i.range = current_range;
                if (!previous_valid) {
                    cluster_id++;
                } else {
                    const double point_distance = std::sqrt(
                        current_range*current_range + previous_range*previous_range -
                        2.0*current_range*previous_range*std::cos(angle_increment));
                    if (point_distance > dist_threshold) {
                        cluster_id++;
                    }
                }
                range_i.cluster = cluster_id;
            }
            lidar.ranges.push_back(range_i);
            previous_valid = current_valid;
            previous_range = current_range;
        }
    
        const double scan_span = std::abs(angle_increment) * static_cast<double>(n - 1);
        const bool is_full_revolution =
            std::abs(scan_span - 2.0*PI) <= 2.0*std::abs(angle_increment);
        if(is_full_revolution && n > 1 && lidar.ranges.front().cluster == 0 &&
            lidar.ranges.back().cluster >= 0)
        {
            const double first_range = lidar.ranges.front().range;
            const double last_range = lidar.ranges.back().range;
            const double wrap_distance = std::sqrt(
                first_range*first_range + last_range*last_range -
                2.0*first_range*last_range*std::cos(angle_increment));
            if (wrap_distance <= dist_threshold) {
                const int last_cluster = lidar.ranges.back().cluster;
                for(auto & range : lidar.ranges) {
                    if(range.cluster == last_cluster) {
                        range.cluster = 0;
                    }
                }
                if(cluster_id > 0) {
                    cluster_id--;
                }
            }
        }

        lidar.n_clusters = cluster_id + 1;
        return lidar;
    }

    std::vector<Vector2D> centroid_finder(Clusters cluster)
    {
        std::vector<Vector2D> centroids{};
        double elements{0.0};

        for(int j{0}; j < (int) cluster.n_clusters; j++)
        {
            Vector2D center{};
            elements = 0.0;

            for(int i{0}; i < (int) cluster.ranges.size(); i++)
            {
                if(cluster.ranges.at(i).cluster == j)
                {
                    center.x += cluster.ranges.at(i).range * cos(cluster.ranges.at(i).angle);
                    center.y += cluster.ranges.at(i).range * sin(cluster.ranges.at(i).angle);
                    elements += 1.0;
                }
            }
            if (elements > 3.0)
            {    
                center.x /= elements;
                center.y /= elements; 
            }
            else{
                center.x = 0.0;
                center.y = 0.0;
            }
            centroids.push_back(center);
        }

        return centroids;
    }

    ClustersCentroids shift_points(Clusters cluster, std::vector<Vector2D> centroids)
    {
        std::vector<std::vector<arma::vec>> cluster_pts(cluster.n_clusters);
        ClustersCentroids clusters_dropped{};

        std::vector<int> point_counts(cluster.n_clusters, 0);
        for (const auto & range : cluster.ranges) {
            if (range.cluster >= 0 && range.cluster < cluster.n_clusters &&
                !(centroids.at(range.cluster).x == 0.0 && centroids.at(range.cluster).y == 0.0))
            {
                point_counts.at(range.cluster)++;
            }
        }

        for (int i{0}; i < cluster.n_clusters; i++)
        {
            arma::vec x(point_counts.at(i), arma::fill::zeros);
            arma::vec y(point_counts.at(i), arma::fill::zeros);
            cluster_pts.at(i).push_back(x);
            cluster_pts.at(i).push_back(y);
        }
        
        std::vector<int> point_index(cluster.n_clusters, 0);

        for(int i{0}; i < (int) cluster.ranges.size(); i++)
        {
            int cluster_index = cluster.ranges.at(i).cluster;
            if(cluster_index >= 0 && cluster_index < cluster.n_clusters)
            {
                if(centroids.at(cluster_index).x == 0.0 && centroids.at(cluster_index).y == 0.0)
                {
                    continue;
                }
                int p_i = point_index.at(cluster_index);
                
                cluster_pts.at(cluster_index).at(0)(p_i) = cluster.ranges.at(i).range * cos(cluster.ranges.at(i).angle) - centroids.at(cluster_index).x;
                cluster_pts.at(cluster_index).at(1)(p_i) = cluster.ranges.at(i).range * sin(cluster.ranges.at(i).angle) - centroids.at(cluster_index).y;

                point_index.at(cluster_index)++;
            }
        }

        Vector2D centroid_i{};
        for(int i{0}; i < cluster.n_clusters; i++)
        {
            if(centroids.at(i).x == 0.0 && centroids.at(i).y == 0.0){
                continue;
            }
            else
            {
                centroid_i.x = centroids.at(i).x;
                centroid_i.y = centroids.at(i).y;
                clusters_dropped.centroids.push_back(centroid_i);
                clusters_dropped.points.push_back(cluster_pts.at(i));
            }
        }

        return clusters_dropped;
    }

    std::vector<Circle> circle_detection(ClustersCentroids clusters)
    {
        std::vector<Circle> circles{};

        for(int i{0}; i < (int) clusters.points.size(); i++)
        {
            arma::vec x = clusters.points.at(i).at(0);
            arma::vec y = clusters.points.at(i).at(1);
            if (x.n_elem < 4) continue;

            arma::vec z = x%x + y%y;
            double z_bar = mean(z);

            arma::mat bigZ(z.n_elem, 4);
            for (int j{0}; j < (int) z.n_elem; j++)
            {
                bigZ(j,0) = z(j);
                bigZ(j,1) = x(j);
                bigZ(j,2) = y(j);
                bigZ(j,3) = 1.0;
            }
            
            arma::mat Hinv = {{0.0, 0.0, 0.0, 0.5},
                              {0.0, 1.0, 0.0, 0.0},
                              {0.0, 0.0, 1.0, 0.0},
                              {0.5, 0.0, 0.0, -2.0*z_bar}};

            arma::mat U, V, A, Y, Sigma, Q, eigvec;
            arma::vec s, A_star, eigval;

            svd(U, s, V, bigZ);
            
            if(s(3) < 1e-12)
            {
                A = V.col(3);
            }
            else
            {
                Sigma = {{s(0), 0.0, 0.0, 0.0},
                         {0.0, s(1), 0.0, 0.0},
                         {0.0, 0.0, s(2), 0.0},
                         {0.0, 0.0, 0.0, s(3)}};

                Y = V*Sigma*V.t();
                Q = Y*Hinv*Y;
                arma::eig_sym(eigval, eigvec, Q);
                
                int eig_index{-1};
                for(int k{0}; k < (int) eigval.n_elem; k++)
                {
                    if(eigval(k) > 0.0)
                    {
                        if(eig_index == -1 || eigval(k) < eigval(eig_index))
                        {
                            eig_index = k;
                        }
                    }
                }
                if (eig_index != -1) {
                    A_star = eigvec.col(eig_index);
                    A = arma::solve(Y, A_star);
                } else {
                    A = V.col(3);
                }
            }

            Circle c{};
            if (std::abs(A(0)) > 1e-9) {
                c.a = -A(1)/(2*A(0)) + clusters.centroids.at(i).x;
                c.b = -A(2)/(2*A(0)) + clusters.centroids.at(i).y;
                double r_sq = (A(1)*A(1) + A(2)*A(2) - 4*A(0)*A(3))/(4*A(0)*A(0));
                c.R = (r_sq > 0.0) ? sqrt(r_sq) : 0.0;
            }

            circles.push_back(c);
        }
        
        return circles;
    }

    std::vector<bool> classification(std::vector<Circle> detected_circles)
    {
        std::vector<bool> is_circle{};

        for (int i{0}; i < (int) detected_circles.size(); i++)
        {
            if(detected_circles.at(i).R > 0.01 && detected_circles.at(i).R < 0.20)
            {
                is_circle.push_back(true);
            }
            else
            {
                is_circle.push_back(false);
            }
        }
        return is_circle;
    }
}
