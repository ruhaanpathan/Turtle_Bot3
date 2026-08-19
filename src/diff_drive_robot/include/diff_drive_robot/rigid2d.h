#ifndef DIFF_DRIVE_ROBOT_RIGID2D_H
#define DIFF_DRIVE_ROBOT_RIGID2D_H

#include <iosfwd>
#include <cmath>

namespace turtlelib
{
    constexpr double PI = 3.14159265358979323846;

    constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12)
    {
        double diff = d1 - d2;
        return std::abs(diff) < epsilon;
    }

    constexpr double deg2rad(double deg)
    {
        return (deg / 360.0 * 2.0 * PI);
    }

    constexpr double rad2deg(double rad)
    {
        return (rad / (2.0 * PI) * 360.0);
    }

    struct Vector2D
    {
        double x = 0.0;
        double y = 0.0;

        Vector2D& operator+=(const Vector2D& rhs);
        Vector2D& operator-=(const Vector2D& rhs);
    };

    double dot(Vector2D& vec1, Vector2D& vec2);
    double magnitude(Vector2D& vec);
    double angle(Vector2D& vec1, Vector2D& vec2);
    Vector2D normalize(Vector2D v);

    Vector2D operator+(Vector2D lhs, const Vector2D rhs);
    Vector2D operator-(Vector2D lhs, const Vector2D rhs);
    Vector2D operator*(double lhs, const Vector2D rhs);
    Vector2D operator*(Vector2D lhs, const double rhs);

    double normalize_angle(double rad);

    std::ostream & operator<<(std::ostream & os, const Vector2D & v);
    std::istream & operator>>(std::istream & is, Vector2D & v);

    struct Twist2D
    {
        double w = 0.0;
        Vector2D v;

        Twist2D();
        Twist2D(double w, Vector2D v);

        friend std::ostream & operator<<(std::ostream & os, const Twist2D & tw);
    };

    class Transform2D
    {
    private:
        double t00, t01, t02;
        double t10, t11, t12;
        double t20, t21, t22;

    public:
        Transform2D();
        explicit Transform2D(Vector2D trans);
        explicit Transform2D(double radians);
        Transform2D(Vector2D trans, double rot);

        Vector2D operator()(Vector2D v) const;
        Transform2D inv() const;
        Transform2D & operator*=(const Transform2D & rhs);

        Vector2D translation() const;
        double rotation() const;

        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
        Twist2D operator()(Twist2D V) const;
    };

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
    std::istream & operator>>(std::istream & is, Transform2D & tf);
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw);
    std::istream & operator>>(std::istream & is, Twist2D & tw);

    Transform2D integrate_twist(Twist2D Vb);
}

#endif
