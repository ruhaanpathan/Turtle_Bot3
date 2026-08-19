#ifndef DIFF_DRIVE_ROBOT_DIFF_DRIVE_LIB_H
#define DIFF_DRIVE_ROBOT_DIFF_DRIVE_LIB_H

#include "diff_drive_robot/rigid2d.h"
#include <vector>
#include <random>

namespace turtlelib
{
    struct Config
    {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
    };

    struct Wheel_Vel
    {
        double l = 0.0;
        double r = 0.0;
    };

    class DiffDrive
    {
    private:
        double D;
        double r;
        Transform2D Tb1;
        Transform2D Tb2;

        double hpi00;
        double hpi01;
        double hpi10;
        double hpi11;

    public:
        double phi_l = 0.0;
        double phi_r = 0.0;
        Config q{};

        DiffDrive(double track_width, double radius);
        DiffDrive(double track_width, double radius, Config cfg);

        Wheel_Vel inverse_kin(Twist2D & Vb) const;
        Twist2D forward_kin(double dphi_l, double dphi_r);
        void update_wheel_pose(double dphi_l, double dphi_r);
    };

    double find_angle(double dx, double dy);
    Config collision_detection(Config q, double collision_radius, std::vector<double> obstacles_x, std::vector<double> obstacles_y, double obstacles_r);
    bool check_direction(Config q, double ix, double iy, double max_x, double max_y);
    double range_obstacles(Config q, double range_max, double range_min, std::vector<double> obstacles_x, std::vector<double> obstacles_y, double obstacles_r, double angle);
    double range_walls(Config q, double range_max, double arena_x, double arena_y, double angle);
    std::mt19937 & get_random();
}

#endif
