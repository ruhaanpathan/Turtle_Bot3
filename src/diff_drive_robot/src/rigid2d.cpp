#include "diff_drive_robot/rigid2d.h"
#include <cmath>
#include <vector>
#include <iostream>

namespace turtlelib
{
    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << '[' << v.x << ' ' << v.y << ']';
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        int first{0};
        char x{};
        while (is.peek()!='\n')
        {
            x = is.peek();
            if (x == ' ')
            {
                first = 1;
                is.get(x);
            }
            else if (std::isdigit(x)||x == '-')
            {   
                (first==0)?is>>v.x:is>>v.y;
            }
            else
            {
                is.get(x);
            }
        }
        return is;
    }

    Vector2D& Vector2D::operator+=(const Vector2D& rhs)
    {
        this->x = this->x + rhs.x;
        this->y = this->y + rhs.y;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D rhs)
    {
        lhs+=rhs;
        return lhs;
    }

    Vector2D& Vector2D::operator-=(const Vector2D& rhs)
    {
        this->x = this->x - rhs.x;
        this->y = this->y - rhs.y;
        return *this;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D rhs)
    {
        lhs-=rhs;
        return lhs;
    }

    Vector2D operator*(const double lhs, Vector2D rhs)
    {
        rhs.x*=lhs;
        rhs.y*=lhs;
        return rhs;
    }

    Vector2D operator*(Vector2D lhs, double rhs)
    {
        lhs.x*=rhs;
        lhs.y*=rhs;
        return lhs;
    }

    double dot(Vector2D& vec1, Vector2D& vec2)
    {
        return vec1.x * vec2.x + vec1.y * vec2.y;
    }

    double magnitude(Vector2D& vec)
    {
        return sqrt(vec.x*vec.x + vec.y*vec.y);
    }

    double angle(Vector2D& vec1, Vector2D& vec2){
        const double num{dot(vec1,vec2)};
        const double den{magnitude(vec1) * magnitude(vec2)};
        double rad = acos(num/den);
        return std::abs(rad);
    }

    Transform2D::Transform2D()
        : t00{1.0}, t01{0.0}, t02{0.0},
          t10{0.0}, t11{1.0}, t12{0.0},
          t20{0.0}, t21{0.0}, t22{1.0}
    {
    }

    Transform2D::Transform2D(Vector2D trans)
        : t00{1.0}, t01{0.0}, t02{trans.x},
          t10{0.0}, t11{1.0}, t12{trans.y},
          t20{0.0}, t21{0.0}, t22{1.0}
    {  
    }

    Transform2D::Transform2D(double radians)
        : t00{cos(radians)}, t01{-sin(radians)}, t02{0.0},
          t10{sin(radians)}, t11{cos(radians)}, t12{0.0},
          t20{0.0}, t21{0.0}, t22{1.0}
    {
    }

    Transform2D::Transform2D(Vector2D trans, double rot)
        : t00{cos(rot)}, t01{-sin(rot)}, t02{trans.x},
          t10{sin(rot)}, t11{cos(rot)}, t12{trans.y},
          t20{0.0}, t21{0.0}, t22{1.0}
    {
    }

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        double x = v.x;
        v.x = t00*x + t01*v.y + t02;
        v.y = t10*x + t11*v.y + t12;
        return v;
    }

    Transform2D Transform2D::inv() const
    {
        Transform2D invT{};
        invT.t00 = t00;
        invT.t01 = t10;
        invT.t02 = -t02*t00 - t12*t10;
        invT.t10 = t01;
        invT.t11 = t00;
        invT.t12 = -t12*t00 + t02*t10;
        return invT;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        Transform2D tmp{};
        tmp = *this;

        tmp.t00 = t00*rhs.t00 + t01*rhs.t10 + t02*rhs.t20;
        tmp.t01 = t00*rhs.t01 + t01*rhs.t11 + t02*rhs.t21;
        tmp.t02 = t00*rhs.t02 + t01*rhs.t12 + t02*rhs.t22;

        tmp.t10 = t10*rhs.t00 + t11*rhs.t10 + t12*rhs.t20;
        tmp.t11 = t10*rhs.t01 + t11*rhs.t11 + t12*rhs.t21;
        tmp.t12 = t10*rhs.t02 + t11*rhs.t12 + t12*rhs.t22;

        tmp.t20 = 0.0;
        tmp.t21 = 0.0;
        tmp.t22 = 1.0;

        *this = tmp;
        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        return {t02,t12};
    }

    double Transform2D::rotation() const
    {
        double radians{0.0};
        if((t00 > 0 || t00 < 0) && t10 > 0){
            radians = acos(t00);
        }
        else if (t00 > 0 && t10 < 0){
            radians = asin(t10);
        }
        else if (t00 < 0 && t10 < 0){
            radians = -acos(t00);
        }
        else if (almost_equal(t00,1.0) && almost_equal(t10, 0.0)){
            radians = 0.0;
        }
        else if (almost_equal(t00,-1.0) && almost_equal(t10, 0.0)){
            radians = PI;
        }
        else if (almost_equal(t00,0.0) && almost_equal(t10, 1.0)){
            radians = PI/2.0;
        }
        else if (almost_equal(t00,0.0) && almost_equal(t10, -1.0)){
            radians = 3*PI/2.0;
        }

        return radians;
    }

    Twist2D Transform2D::operator()(Twist2D V) const
    {
        Twist2D V_new{};
        std::vector<std::vector<double>> adj{{1.0, 0.0, 0.0}
                ,{t12, t00, t01}
                ,{-t02, t10, t11}};
        V_new.w = V.w;
        V_new.v.x = adj.at(1).at(0)*V.w + adj.at(1).at(1)*V.v.x + adj.at(1).at(2)*V.v.y;
        V_new.v.y = adj.at(2).at(0)*V.w + adj.at(2).at(1)*V.v.x + adj.at(2).at(2)*V.v.y;
        return V_new;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        Vector2D vec{};
        double deg{};
        int space_pos{0};
        char x{};
        int num{0};
        while (is.peek()!='\n')
        {
            x = is.peek();
            if (x == ' ')
            {
                space_pos = space_pos + 1;
                is.get(x);
            }
            else if (std::isdigit(x)||x == '-')
            {   
                if(space_pos==0)
                {
                    num = 1;
                    is >> deg;
                    space_pos = 2;
                }
                else if(space_pos==1){is>>deg;}
                else if(space_pos==3)
                {
                    is>>vec.x;
                    if(num==1){space_pos = 4;}
                }
                else if(space_pos==5){is>>vec.y;}
            }
            else
            {
                is.get(x);
            }
        }
        x = is.peek();
        is.get(x);
        auto radians = deg2rad(deg);
        Transform2D newtf(vec, radians);
        tf = newtf;
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        lhs*=rhs;
        return lhs;
    }

    Vector2D normalize(Vector2D v)
    {
        double mag = sqrt(v.x*v.x + v.y*v.y);
        if (mag > 1e-9) {
            v.x /= mag;
            v.y /= mag;
        }
        return v;
    }

    double normalize_angle(double rad)
    {
        double deg = rad2deg(rad);
        return deg2rad(remainder(deg, 360.0));
    }

    Twist2D::Twist2D()
        : w{0}, v{0,0}
    {
    }

    Twist2D::Twist2D(double ang_v, Vector2D xy)
        : w{ang_v}, v{xy}
    {
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
    {
        os << "[" << tw.w << " " << tw.v.x << " " << tw.v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw)
    {
        Vector2D v{};
        double w{0.0};
        int space_pos{0};
        char x{};
        x = is.peek();
        is.get(x);
        while (is.peek()!='\n')
        {
            x = is.peek();
            if (x == ' ')
            {
                space_pos += 1;
                is.get(x);
            }
            else if (std::isdigit(x)||x == '-')
            {   
                if (space_pos == 0)
                {
                    is >> w;
                }
                else if (space_pos == 1)
                {
                    is >> v.x;
                }
                else if (space_pos == 2)
                {
                    is >> v.y;
                }
            }
            else
            {
                is.get(x);
            }
        }
        x = is.peek();
        is.get(x);
        Twist2D tmp{w, v};
        tw = tmp;
        return is;
    }

    Transform2D integrate_twist(Twist2D Vb)
    {
        if (almost_equal(Vb.w,0))
        {
            const Transform2D Tb_bp({Vb.v.x, Vb.v.y}, 0.0);
            return Tb_bp;
        }
        else{
            // Translate to the instantaneous center of rotation, rotate, then
            // translate back. Ts_b is a pure translation.
            const Transform2D Ts_b({Vb.v.y/Vb.w, -Vb.v.x/Vb.w});
            const Transform2D Ts_sp(Vb.w);
            const Transform2D Tb_bp = Ts_b.inv() * Ts_sp * Ts_b;
            return Tb_bp;
        }
    }
}
