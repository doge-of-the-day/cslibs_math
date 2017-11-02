#ifndef BOX_2D_HPP
#define BOX_2D_HPP

#include <cslibs_math_2d/types/line.hpp>

#include <limits>
#include <set>

namespace cslibs_math_2d {
class Box2d
{
public:
    using point_set_t    = std::set<Point2d>;
    using coefficients_t = std::array<double, 2>;

    inline Box2d() :
        min_(std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest()),
        max_(std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max())
    {
    }

    inline Box2d(const double min_x, const double min_y,
                 const double max_x, const double max_y) :
        min_(min_x, min_y),
        max_(max_x, max_y)
    {
    }

    inline Box2d(const Point2d &min,
                 const Point2d &max) :
        min_(min),
        max_(max)
    {
    }

    inline void setMin(const Point2d &min)
    {
        min_ = min;
    }

    inline void setMax(const Point2d &max)
    {
        max_ = max;
    }

    inline Point2d const & getMin() const
    {
        return min_;
    }

    inline Point2d const & getMax() const
    {
        return max_;
    }

    inline Point2d lu() const
    {
        return Point2d(min_.x(), max_.y());
    }

    inline Point2d ll() const
    {
        return min_;
    }

    inline Point2d ru() const
    {
        return max_;
    }

    inline Point2d rl() const
    {
        return Point2d(max_.x(), min_.y());
    }

    inline bool intersects(const Line2d &line) const
    {
        //// LIANG BARSKY
        const auto p0 = line[0];
        const auto p1 = line[1];
        const auto d = p1 - p0;

        double t0 = 0.0;
        double t1 = 1.0;

        auto clip = [] (const double p, const double q,
                        double &t0, double &t1)
        {
            if(p == 0 && q < 0)
                return false;

            const double r = q / p;
            if(p < 0) {
                if(r > t1)
                    return false;
                t0 = r > t0 ? r : t0;
            }
            if(p > 0) {
                if(r < t0)
                    return false;
                t1 = r < t1 ? r : t1;
            }
            return true;
        };

        if(!clip(-d.x(), -(min_.x()-p0.x()), t0, t1))
                return false;

        if(!clip(d.x(), (max_.x()-p0.x()), t0, t1))
                return false;

        if(!clip(-d.y(), -(min_.y()-p0.y()), t0, t1))
                return false;

        if(!clip(d.y(), (max_.y()-p0.y()), t0, t1))
                return false;
        return true;
    }

    inline bool intersection(const Line2d &line,
                             Line2d &clipped)
    {
        const auto p0 = line[0];
        const auto p1 = line[1];
        const auto d = p1 - p0;

        double t0 = 0.0;
        double t1 = 1.0;

        auto clip = [] (const double p, const double q,
                        double &t0, double &t1)
        {
            if(p == 0 && q < 0)
                return false;

            const double r = q / p;
            if(p < 0) {
                if(r > t1)
                    return false;
                t0 = r > t0 ? r : t0;
            }
            if(p > 0) {
                if(r < t0)
                    return false;
                t1 = r < t1 ? r : t1;
            }
            return true;
        };

        if(!clip(-d.x(), -(min_.x()-p0.x()), t0, t1))
                return false;

        if(!clip(d.x(), (max_.x()-p0.x()), t0, t1))
                return false;

        if(!clip(-d.y(), -(min_.y()-p0.y()), t0, t1))
                return false;

        if(!clip(d.y(), (max_.y()-p0.y()), t0, t1))
                return false;

        clipped[0].x() = p0.x() + t0*d.x();
        clipped[0].y() = p0.y() + t0*d.y();
        clipped[1].x() = p0.x() + t1*d.x();
        clipped[1].y() = p0.y() + t1*d.y();

        return true;
    }

private:
    Point2d min_;
    Point2d max_;

}__attribute__ ((aligned (32)));
}

inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Box2d &b)
{
    out << "[" << b.getMin() << "," << b.getMax() << "]";
    return out;
}

#endif // BOX_2D_HPP
