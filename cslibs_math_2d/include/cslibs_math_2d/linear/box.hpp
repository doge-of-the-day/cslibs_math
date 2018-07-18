#ifndef CSLIBS_MATH_2D_BOX_2D_HPP
#define CSLIBS_MATH_2D_BOX_2D_HPP

#include <cslibs_math_2d/linear/line.hpp>

#include <cslibs_math/common/equal.hpp>

#include <limits>
#include <set>

namespace cslibs_math_2d {
class Box2d
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using allocator_t     = Eigen::aligned_allocator<Box2d>;
    using point_set_t    = std::set<Point2d, Point2d::allocator_t>;
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
        return Point2d(min_(0), max_(1));
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
        return Point2d(max_(0), min_(1));
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
            if(cslibs_math::common::eq(p, 0.0) && q < 0.0)
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

        if(!clip(-d(0), -(min_(0)-p0(0)), t0, t1))
                return false;

        if(!clip(d(0), (max_(0)-p0(0)), t0, t1))
                return false;

        if(!clip(-d(1), -(min_(1)-p0(1)), t0, t1))
                return false;

        if(!clip(d(1), (max_(1)-p0(1)), t0, t1))
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
            if(cslibs_math::common::eq(p, 0.0) && q < 0.0)
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

        if(!clip(-d(0), -(min_(0)-p0(0)), t0, t1))
                return false;

        if(!clip(d(0), (max_(0)-p0(0)), t0, t1))
                return false;

        if(!clip(-d(1), -(min_(1)-p0(1)), t0, t1))
                return false;

        if(!clip(d(1), (max_(1)-p0(1)), t0, t1))
                return false;

        clipped[0](0) = p0(0) + t0*d(0);
        clipped[0](1) = p0(1) + t0*d(1);
        clipped[1](0) = p0(0) + t1*d(0);
        clipped[1](1) = p0(1) + t1*d(1);

        return true;
    }

private:
    Point2d min_;
    Point2d max_;
};
}

inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Box2d &b)
{
    out << "[" << b.getMin() << "," << b.getMax() << "]";
    return out;
}

#endif // CSLIBS_MATH_2D_BOX_2D_HPP
