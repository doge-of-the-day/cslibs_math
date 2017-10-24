#ifndef VECTOR_2D_HPP
#define VECTOR_2D_HPP

#include <cmath>
#include <eigen3/Eigen/Core>

namespace cslibs_math_2d {
class Vector2d {
public:
    inline Vector2d() :
        x_(0.0),
        y_(0.0)
    {
    }

    inline Vector2d(const double x,
                    const double y) :
        x_(x),
        y_(y)
    {
    }

    inline Vector2d(const Vector2d &other) :
        x_(other.x_),
        y_(other.y_)
    {
    }

    inline Vector2d(Vector2d &&other) :
        x_(other.x_),
        y_(other.y_)
    {
    }

    inline Vector2d operator * (const double d) const
    {
        return Vector2d(x_ * d, y_ * d);
    }

    inline Vector2d operator / (const double d) const
    {
        return Vector2d(x_ / d, y_ / d);
    }

    inline Vector2d operator + (const Vector2d &other) const
    {
        return Vector2d(x_ + other.x_, y_ + other.y_);
    }

    inline Vector2d operator - (const Vector2d &other) const
    {
        return Vector2d(x_ - other.x_, y_ - other.y_);
    }

    inline double dot (const Vector2d &other) const
    {
        return x_ * other.x_ + y_ * other.y_;
    }

    inline double length () const
    {
        return sqrt(length2());
    }

    inline double length2() const
    {
        return x_ * x_ + y_ * y_;
    }

    inline double angle() const
    {
        return atan2(y_, x_);
    }

    inline double & x()
    {
        return x_;
    }

    inline double & y()
    {
        return y_;
    }

    inline double const & x() const
    {
        return x_;
    }

    inline double const & y() const
    {
        return y_;
    }

    inline Vector2d & operator += (const Vector2d &other)
    {
        x_ += other.x_;
        y_ += other.y_;
        return *this;
    }

    inline Vector2d & operator -= (const Vector2d &other)
    {
        x_ -= other.x_;
        y_ -= other.y_;
        return *this;
    }

    inline Vector2d & operator *= (const double d)
    {
        x_ *= d;
        y_ *= d;
        return *this;
    }

    inline Vector2d & operator /= (const double d)
    {
        x_ /= d;
        y_ /= d;
        return *this;
    }

    inline Vector2d& operator = (const Vector2d &other)
    {
        x_ = other.x_;
        y_ = other.y_;
        return *this;
    }

    inline Vector2d& operator = (const Eigen::Vector2d &other)
    {
        x_ = other(0);
        y_ = other(1);
        return *this;
    }

    inline Vector2d& operator = (Vector2d &&other)
    {
        x_ = other.x_;
        y_ = other.y_;
        return *this;
    }

    inline Vector2d normalized() const
    {
        const double len = length();
        return Vector2d(x_ / len, y_ / len);
    }

    inline Vector2d operator -() const
    {
        return Vector2d(-x_, -y_);
    }

    inline Eigen::Vector2d toEigen() const
    {
        return Eigen::Vector2d(x_, y_);
    }

    inline Vector2d min(const Vector2d &other) const
    {
        return Vector2d(fmin(x_, other.x_),
                        fmin(y_, other.y_));
    }

    inline Vector2d max(const Vector2d &other) const
    {
        return Vector2d(fmax(x_, other.x_),
                        fmax(y_, other.y_));
    }

    inline double distance(const Vector2d &other) const
    {
        return  hypot(x_ - other.x_, y_ - other.y_);
    }

    inline double distance2(const Vector2d &other) const
    {
        return  hypot2(x_ - other.x_, y_ - other.y_);
    }

    inline bool isNormal() const
    {
        return std::isnormal(x_) && std::isnormal(y_);
    }


private:
    static inline double hypot2(const double x, const double y)
    {
        return x*x + y*y;
    }

    static inline double hypot (const double x, const double y)
    {
        return sqrt(x*x + y*y);
    }

    double x_;
    double y_;
} __attribute__ ((aligned (16)));
}

inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Vector2d &v)
{
    out << "[" << v.x() << "," << v.y() << "]";
    return out;
}

#endif // VECTOR_2D_HPP
