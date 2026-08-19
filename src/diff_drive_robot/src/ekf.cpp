#include "diff_drive_robot/ekf.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace turtlelib
{
    EKF::EKF(Config q_0, int num_obst, double process_cov, double r)
    : sigma_est(3+2*num_obst, 3+2*num_obst, arma::fill::zeros), n{num_obst},
      Q(3,3, arma::fill::eye), Q_bar(2*num_obst + 3, 2*num_obst + 3, arma::fill::zeros),
      R(2,2, arma::fill::eye), I(3+2*num_obst, 3+2*num_obst, arma::fill::eye),
      q_prev{q_0}, zeta_est(2*num_obst + 3, arma::fill::zeros),
      A_c(2*num_obst + 3, 2*num_obst + 3, arma::fill::eye)
    {
        if (n <= 0 || process_cov < 0.0 || r <= 0.0) {
            throw std::invalid_argument(
                "EKF landmark count and covariance parameters must be valid");
        }
        for(int i{0}; i < n; i++)
        {
            izd.push_back(false);
            obst_radii.push_back(0.0);
        }

        arma::mat sigma_0m(2*n, 2*n, arma::fill::zeros);
        double d = 1e10;
        sigma_0m.diag().fill(d);

        sigma_est.submat( 3,3, (3+2*n-1),(3+2*n-1) ) = sigma_0m;

        Q = process_cov * Q;
        Q_bar.submat( 0,0, 2,2 ) = Q;
        R = r*R;
    }

    void EKF::prediction(Config q)
    {
        zeta_prev = zeta_est;
        sigma_prev = sigma_est;

        const double odom_dx = q.x - q_prev.x;
        const double odom_dy = q.y - q_prev.y;
        const double delta_theta = normalize_angle(q.theta - q_prev.theta);

        // Express the odometry increment in the previous robot frame, then
        // compose that body-frame motion with the current map-frame estimate.
        const double body_dx = std::cos(q_prev.theta)*odom_dx +
            std::sin(q_prev.theta)*odom_dy;
        const double body_dy = -std::sin(q_prev.theta)*odom_dx +
            std::cos(q_prev.theta)*odom_dy;
        const double map_dx = std::cos(zeta_prev(0))*body_dx -
            std::sin(zeta_prev(0))*body_dy;
        const double map_dy = std::sin(zeta_prev(0))*body_dx +
            std::cos(zeta_prev(0))*body_dy;

        zeta_est(0) = normalize_angle(zeta_prev(0) + delta_theta);
        zeta_est(1) = zeta_prev(1) + map_dx;
        zeta_est(2) = zeta_prev(2) + map_dy;

        arma::mat A(3,3, arma::fill::eye);
        A(1,0) = -map_dy;
        A(2,0) = map_dx;
        A_c.submat(0,0, 2,2) = A;

        q_prev = q;

        sigma_est = A_c*sigma_prev*A_c.t() + Q_bar;
    }

    void EKF::initialization(int index, double dx, double dy)
    {
        int m_index = 3 + 2*index;
        const double r = sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        const double phi = atan2(dy,dx);
        const double alpha = normalize_angle(phi + zeta_est(0));

        zeta_est(m_index) = zeta_est(1) + r*cos(alpha);
        zeta_est(m_index+1) = zeta_est(2) + r*sin(alpha);

        // Standard EKF-SLAM state augmentation. This replaces the large
        // placeholder prior with covariance derived from robot and sensor
        // uncertainty, including all robot/landmark cross-correlations.
        arma::mat robot_jacobian = {
            {-r*sin(alpha), 1.0, 0.0},
            { r*cos(alpha), 0.0, 1.0}
        };
        arma::mat measurement_jacobian = {
            {cos(alpha), -r*sin(alpha)},
            {sin(alpha),  r*cos(alpha)}
        };
        const arma::mat landmark_cross =
            robot_jacobian * sigma_est.rows(0, 2);
        sigma_est.rows(m_index, m_index + 1) = landmark_cross;
        sigma_est.cols(m_index, m_index + 1) = landmark_cross.t();
        sigma_est.submat(m_index, m_index, m_index + 1, m_index + 1) =
            robot_jacobian * sigma_est.submat(0, 0, 2, 2) * robot_jacobian.t() +
            measurement_jacobian * R * measurement_jacobian.t();
        sigma_est = 0.5*(sigma_est + sigma_est.t());
    }

    void EKF::correction(int index, double dx, double dy)
    {
        const arma::vec state_before = zeta_est;
        const arma::mat covariance_before = sigma_est;
        int m_index = 3 + 2*index;
        double r = sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        double phi = atan2(dy,dx);

        arma::vec z_real = {r, phi};

        double delta_x = zeta_est(m_index) - zeta_est(1);
        double delta_y = zeta_est(m_index+1) - zeta_est(2);
        double d = delta_x*delta_x + delta_y*delta_y;
        if (d < 1e-9) d = 1e-9;

        arma::vec z_theor = {sqrt(d), normalize_angle(atan2(delta_y, delta_x) - zeta_est(0))};

        arma::mat H(2, 3+2*n, arma::fill::zeros);

        arma::mat h_1 = {{0.0, -delta_x/sqrt(d), -delta_y/sqrt(d)},
                         {-1.0, delta_y/d, -delta_x/d}};

        arma::mat h_2 = {{delta_x/sqrt(d), delta_y/sqrt(d)},
                         {-delta_y/d, delta_x/d}};

        H.submat(0,0, 1,2) = h_1;
        H.submat(0,m_index, 1,m_index+1) = h_2;

        const arma::mat innovation_covariance = H*sigma_est*H.t() + R;
        arma::mat solved;
        if (!arma::solve(
                solved, innovation_covariance, H*sigma_est,
                arma::solve_opts::likely_sympd))
        {
            return;
        }
        const arma::mat K = solved.t();

        arma::vec innovation = z_real - z_theor;
        innovation(1) = normalize_angle(innovation(1));
        if (!innovation.is_finite()) return;
        zeta_est = zeta_est + K*innovation;
        if (!zeta_est.is_finite()) {
            zeta_est = state_before;
            sigma_est = covariance_before;
            return;
        }
        zeta_est(0) = normalize_angle(zeta_est(0));

        // Joseph form preserves symmetry and positive semi-definiteness better.
        const arma::mat correction_matrix = I - K*H;
        sigma_est = correction_matrix*sigma_est*correction_matrix.t() + K*R*K.t();
        sigma_est = 0.5*(sigma_est + sigma_est.t());
    }

    double EKF::mah_distance(const Circle & lmark, int k) const
    {
        int m_index = 3 + 2*k;
        double r = sqrt(std::pow(lmark.a, 2) + std::pow(lmark.b, 2));
        double phi = atan2(lmark.b,lmark.a);

        arma::vec z_real = {r, phi};

        double delta_x = zeta_est(m_index) - zeta_est(1);
        double delta_y = zeta_est(m_index+1) - zeta_est(2);
        double d = delta_x*delta_x + delta_y*delta_y;
        if (d < 1e-9) d = 1e-9;

        arma::mat H(2, 3+2*n, arma::fill::zeros);

        arma::mat h_1 = {{0.0, -delta_x/sqrt(d), -delta_y/sqrt(d)},
                         {-1.0, delta_y/d, -delta_x/d}};

        arma::mat h_2 = {{delta_x/sqrt(d), delta_y/sqrt(d)},
                         {-delta_y/d, delta_x/d}};

        H.submat(0,0, 1,2) = h_1;
        H.submat(0,m_index, 1,m_index+1) = h_2;

        arma::mat Psi = H * sigma_est * H.t() + R;
        arma::vec z_theor = {sqrt(d), normalize_angle(atan2(delta_y, delta_x) - zeta_est(0))};

        arma::vec innovation = z_real - z_theor;
        innovation(1) = normalize_angle(innovation(1));
        arma::vec weighted;
        if (!innovation.is_finite() ||
            !arma::solve(weighted, Psi, innovation, arma::solve_opts::likely_sympd))
        {
            return std::numeric_limits<double>::infinity();
        }
        return arma::dot(innovation, weighted);
    }
}
