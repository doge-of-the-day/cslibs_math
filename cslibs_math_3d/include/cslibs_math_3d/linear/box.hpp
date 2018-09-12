#ifndef CSLIBS_MATH_3D_BOX_3D_HPP
#define CSLIBS_MATH_3D_BOX_3D_HPP

#include <cslibs_math_3d/linear/line.hpp>

#include <cslibs_math/common/equal.hpp>

#include <limits>
#include <set>

namespace cslibs_math_3d {
class EIGEN_ALIGN16 Box3d
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Box3d>;

    using point_set_t    = std::set<Point3d, Point3d::allocator_t>;
    using coefficients_t = std::array<double, 3>;

    inline Box3d() :
        min_(std::numeric_limits<double>::lowest()),
        max_(std::numeric_limits<double>::max())
    {
    }

    inline Box3d(const double min_x, const double min_y, const double min_z,
                 const double max_x, const double max_y, const double max_z) :
        min_(min_x, min_y, min_z),
        max_(max_x, max_y, max_z)
    {
    }

    inline Box3d(const Point3d &min,
                 const Point3d &max) :
        min_(min),
        max_(max)
    {
    }

    inline Box3d(const Box3d &other) :
        min_(other.min_),
        max_(other.max_)
    {
    }

    inline Box3d(Box3d &&other) :
        min_(std::move(other.min_)),
        max_(std::move(other.max_))
    {
    }

    inline void setMin(const Point3d &min)
    {
        min_ = min;
    }

    inline void setMax(const Point3d &max)
    {
        max_ = max;
    }

    inline Point3d const & getMin() const
    {
        return min_;
    }

    inline Point3d const & getMax() const
    {
        return max_;
    }


    inline Point3d ll() const
    {
        return min_;
    }

    inline Point3d ru() const
    {
        return max_;
    }

    inline bool intersects(const Line3d &line) const
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

    inline bool intersection(const Line3d &line,
                             Line3d &clipped)
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
    Point3d min_;
    Point3d max_;

};
}

inline std::ostream & operator << (std::ostream &out, const cslibs_math_3d::Box3d &b)
{
    out << "[" << b.getMin() << "," << b.getMax() << "]";
    return out;
}

#endif // CSLIBS_MATH_3D_BOX_2D_HPP
