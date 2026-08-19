#include "diff_drive_robot/diff_drive_lib.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace turtlelib
{
    DiffDrive::DiffDrive(double track_width, double radius)
     :  D{track_width/2.0}, r{radius},
        Tb1{{0.0,D}, 0.0},
        Tb2{{0.0, -D}, 0.0},
        hpi00{(radius)/(-2.0*D)}, hpi01{(radius)/(2.0*D)}, hpi10{radius/2.0}, hpi11{radius/2.0},
        phi_l{0.0}, phi_r{0.0},
        q{0.0,0.0,0.0}
    {
    }

    DiffDrive::DiffDrive(double track_width, double radius, Config cfg)
     :  D{track_width/2.0}, r{radius},
        Tb1{{0.0,D}, 0.0},
        Tb2{{0.0, -D}, 0.0},
        hpi00{(radius)/(-2.0*D)}, hpi01{(radius)/(2.0*D)}, hpi10{radius/2.0}, hpi11{radius/2.0},
        phi_l{0.0}, phi_r{0.0},
        q{cfg.x,cfg.y,cfg.theta}
    {
    }

    Wheel_Vel DiffDrive::inverse_kin(Twist2D & Vb) const{
        if (!almost_equal(Vb.v.y,0)){
            throw std::logic_error("y component of body twist must be zero");
        }

        Wheel_Vel phidot{};
        phidot.l = (-D*Vb.w + Vb.v.x)/r;
        phidot.r = (D*Vb.w + Vb.v.x)/r;
        return phidot;
    }

    Twist2D DiffDrive::forward_kin(double dphi_l, double dphi_r){
        Twist2D Vb{};

        Vb.w = hpi00*dphi_l + hpi01*dphi_r;
        Vb.v.x = hpi10*dphi_l + hpi11*dphi_r;
        Vb.v.y = 0.0;
        
        Transform2D Tb_bp = integrate_twist(Vb);
        Config dq{Tb_bp.translation().x, Tb_bp.translation().y, Tb_bp.rotation()};
        const double theta_prev = q.theta;
        q.x += dq.x*cos(theta_prev) - dq.y*sin(theta_prev);
        q.y += dq.x*sin(theta_prev) + dq.y*cos(theta_prev);
        q.theta = normalize_angle(theta_prev + dq.theta);

        return Vb;
    }

    void DiffDrive::update_wheel_pose(double dphi_l, double dphi_r)
    {
        phi_l += dphi_l;
        phi_r += dphi_r;
    }

    double find_angle(double dx, double dy)
    {
        double radians{0};
        double mag = sqrt(std::pow(dx,2) + std::pow(dy,2));
        if (mag < 1e-9) return 0.0;
        if(dy > 0){
            radians = acos(dx/mag);
        }
        else if (dx > 0 && dy < 0){
            radians = asin(dy/mag);
        }
        else if (dx < 0 && dy < 0){
            radians = -acos(dx/mag);
        }
        else if (dx > 0 && almost_equal(dy, 0.0)){
            radians = 0.0;
        }
        else if (almost_equal(dx,0.0) && dy > 0){
            radians = PI/2;
        }
        else if (almost_equal(dy,0.0) && dx < 0){
            radians = PI;
        }
        else if (almost_equal(dx,0.0) && dy < 0){
            radians = 3*PI/2.0;
        }
        return radians;
    }

    Config collision_detection(Config q, double collision_radius, std::vector<double> obstacles_x, std::vector<double> obstacles_y, double obstacles_r)
    {
        const auto R{collision_radius + obstacles_r};
        for(int i = 0; i < (int) obstacles_x.size(); i++)
        {   
            const double dx{q.x - obstacles_x.at(i)};
            const double dy{q.y - obstacles_y.at(i)};
            const auto dr = sqrt(std::pow(dx,2) + std::pow(dy,2));
            if(dr<R)
            {   
                const auto angle = find_angle(dx, dy);
                const auto xp = obstacles_x.at(i) + R*cos(angle);
                const auto yp = obstacles_y.at(i) + R*sin(angle);
                q.x = xp;
                q.y = yp;
                break;
            }
        }
        return q;
    }

    bool check_direction(Config q, double ix, double iy, double max_x, double max_y)
    {
        double check1{};
        double check2{};
        if(almost_equal(max_y,q.y))
        {
            check2 = 0.0;
        }
        else
        {
            check2 = (iy - q.y)/(max_y - q.y);
        }
        
        if(almost_equal(max_x, q.x))
        {
            check1 = 0.0;
        }
        else
        {
            check1 = (ix - q.x)/(max_x - q.x);
        }

        return (check1 >= 0.0 && check2 >= 0.0);
    }

    double range_obstacles(Config q, double range_max, double range_min, std::vector<double> obstacles_x, std::vector<double> obstacles_y, double obstacles_r, double angle)
    {
        double ranges{0.0};
        bool exists{false};
        double sx{}; double sy{};
        const auto max_x = q.x + range_max*cos(angle + q.theta);
        const auto max_y = q.y + range_max*sin(angle + q.theta);
        const auto max_range = sqrt(std::pow(range_max*cos(angle + q.theta), 2.0) + std::pow(range_max*sin(angle + q.theta), 2.0));
        const auto min_range = sqrt(std::pow(range_min*cos(angle + q.theta), 2.0) + std::pow(range_min*sin(angle + q.theta), 2.0));
        const auto m = (max_y - q.y) / (max_x - q.x);

        for(int j = 0; j < (int) obstacles_x.size(); j++)
        {   
            double ix{}; double iy{};
            double ix_1{}; double iy_1{};

            if(almost_equal(max_x, q.x))
            {
                ix = q.x;
                const auto b = -2*obstacles_y.at(j);
                const auto c = std::pow(obstacles_y.at(j), 2.0) + ix*ix - 2*ix*obstacles_x.at(j) + std::pow(obstacles_x.at(j), 2.0) - std::pow(obstacles_r, 2.0);
                const auto det = b*b - 4*c;
                if(det < 0.0)
                {
                    continue;
                }
                else if(almost_equal(det,0.0))
                {
                    iy = -b/2.0;
                }
                else if(det > 0.0)
                {
                    iy = (-b + sqrt(det))/2.0;
                    ix_1 = ix;
                    iy_1 = (-b - sqrt(det))/2.0;

                    if(sqrt(std::pow(ix-q.x, 2.0) + std::pow(iy-q.y, 2.0)) >= sqrt(std::pow(ix_1-q.x, 2.0) + std::pow(iy_1-q.y, 2.0)))
                    {
                        ix = ix_1;
                        iy = iy_1;
                    }
                }
            }
            else
            {
                const auto alpha = q.y - m*q.x - obstacles_y.at(j);
                const auto a = 1 + m*m;
                const auto b = 2*(alpha*m - obstacles_x.at(j));
                const auto c = std::pow(obstacles_x.at(j), 2.0) + alpha*alpha - std::pow(obstacles_r, 2.0);
                const auto det = b*b - 4*a*c;
                if(det < 0.0)
                {
                    continue;
                }
                else if(almost_equal(det,0.0))
                {
                    ix = -b / (2*a);
                    iy = m*(ix - q.x) + q.y;
                }
                else if(det > 0.0)
                {
                    ix = (-b + sqrt(det))/(2*a);
                    iy = m*(ix - q.x) + q.y;
                    ix_1 = (-b - sqrt(det))/(2*a);
                    iy_1 = m*(ix_1 - q.x) + q.y;

                    if(sqrt(std::pow(ix-q.x, 2.0) + std::pow(iy-q.y, 2.0)) >= sqrt(std::pow(ix_1-q.x, 2.0) + std::pow(iy_1-q.y, 2.0)))
                    {
                        ix = ix_1;
                        iy = iy_1;
                    }
                }
            }

            const auto range = sqrt(std::pow(ix-q.x, 2.0) + std::pow(iy-q.y, 2.0));
            if(range < max_range && range > min_range && check_direction(q, ix, iy, max_x, max_y))
            {
                if(!exists)
                {
                    ranges = range;
                    exists = true;
                    sx = ix;
                    sy = iy;
                }
                else
                {
                    const auto s_range = sqrt(std::pow(sx-q.x, 2.0) + std::pow(sy-q.y, 2.0));
                    if(range < s_range)
                    {
                        ranges = range;
                        sx = ix;
                        sy = iy;
                    }
                    else
                    {
                        ranges = s_range;
                    }
                }
            }
        }
        return ranges;
    }

    double range_walls(Config q, double range_max, double arena_x, double arena_y, double angle)
    {
        std::vector<double> x_pos = {arena_x/2.0, -arena_x/2.0, 0.0, 0.0};
        std::vector<double> y_pos = {0.0, 0.0, arena_y/2.0, -arena_y/2.0};

        double ranges{0.0};
        const auto max_x = q.x + range_max*cos(q.theta + angle);
        const auto max_y = q.y + range_max*sin(angle + q.theta);
        const auto m = (max_y - q.y) / (max_x - q.x);
        bool exists{false};
        
        for(int i = 0; i < 4; i++)
        {
            if(almost_equal(max_x, q.x))
            {
                if(x_pos.at(i) == 0.0)
                {
                    if(std::abs(max_y) >= std::abs(y_pos.at(i)))
                    {   
                        if(max_y > 0 && y_pos.at(i) > 0)
                        {
                            ranges = y_pos.at(i) - q.y;
                        }
                        else if(max_y < 0 && y_pos.at(i) < 0)
                        {
                            ranges = q.y - y_pos.at(i);
                        }
                    }
                }
            }
            else
            {
                double ix{}; double iy{};
                if(x_pos.at(i) != 0.0)
                {
                    if(std::abs(max_x) > std::abs(x_pos.at(i)))
                    {   
                        ix = x_pos.at(i);
                        iy = m*(ix - q.x) + q.y;
                        if(check_direction(q, ix, iy, max_x, max_y))
                        {
                            if(!exists)
                            { 
                                exists = true;
                                ranges = sqrt(std::pow(ix-q.x, 2.0) + std::pow(iy-q.y, 2.0));
                            }
                            else
                            {
                                const auto s_ranges = ranges;
                                ranges = sqrt(std::pow(ix-q.x, 2.0) + std::pow(iy-q.y, 2.0));
                                if(s_ranges < ranges)
                                {
                                    ranges = s_ranges;
                                }
                            }
                        }
                    }
                }
                else
                {
                    if(std::abs(max_y) > std::abs(y_pos.at(i)))
                    {
                        iy = y_pos.at(i);
                        ix = (iy - q.y + m*q.x)/m;
                        if(check_direction(q, ix, iy, max_x, max_y))
                        {
                            if(!exists)
                            {
                                exists = true;
                                ranges = sqrt(std::pow(ix-q.x, 2.0) + std::pow(iy-q.y, 2.0));
                            }
                            else
                            {
                                const auto s_ranges = ranges;
                                ranges = sqrt(std::pow(ix-q.x, 2.0) + std::pow(iy-q.y, 2.0));
                                if(s_ranges < ranges)
                                {
                                    ranges = s_ranges;
                                }
                            }
                        }
                    }
                }
            }
        }
        return ranges;
    }

    std::mt19937 & get_random()
    {
        static std::random_device rd{};
        static std::mt19937 mt{rd()};
        return mt;
    }
}
