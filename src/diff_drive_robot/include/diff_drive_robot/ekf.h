#ifndef DIFF_DRIVE_ROBOT_EKF_H
#define DIFF_DRIVE_ROBOT_EKF_H

#include "diff_drive_robot/rigid2d.h"
#include "diff_drive_robot/diff_drive_lib.h"
#include <armadillo>
#include <vector>

namespace turtlelib
{
    struct Circle
    {
        double a = 0.0;
        double b = 0.0;
        double R = 0.0;
    };

    class EKF
    {
    private:
        arma::vec zeta_prev;
        arma::mat sigma_est;
        arma::mat sigma_prev;
        int n;
        arma::mat Q;
        arma::mat Q_bar;
        arma::mat R;
        arma::mat I;
        Config q_prev;

    public:
        arma::vec zeta_est;
        arma::mat A_c;
        std::vector<bool> izd;
        std::vector<double> obst_radii;

        EKF(Config q_0, int num_obst, double process_cov, double r);

        void prediction(Config q);
        void initialization(int index, double dx, double dy);
        void correction(int index, double dx, double dy);
        double mah_distance(const Circle & lmark, int k) const;
    };
}

#endif
