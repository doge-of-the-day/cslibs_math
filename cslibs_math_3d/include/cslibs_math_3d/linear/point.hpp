#ifndef CSLIBS_MATH_3D_POINT_3D_HPP
#define CSLIBS_MATH_3D_POINT_3D_HPP

#include <cslibs_math/color/color.hpp>
#include <cslibs_math_3d/linear/vector.hpp>

namespace cslibs_math_3d {
template <typename T>
using Point3d = Vector3d<T>;

template <typename T>
class EIGEN_ALIGN16 PointRGB3d //: public Point3d
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<PointRGB3d>;

    using point_t = Point3d<T>;
    using color_t = cslibs_math::color::Color<T>;

    inline PointRGB3d():
        a_(1.0)
    {
    }

    inline PointRGB3d(const PointRGB3d &other) :
        point_(other.point_),
        a_(other.a_),
        color_(other.color_)
    {
    }

    inline PointRGB3d(PointRGB3d &&other) :
        point_(std::move(other.point_)),
        a_(std::move(other.a_)),
        color_(std::move(other.color_))
    {
    }

    inline PointRGB3d(const point_t &pos) :
        point_(pos),
        a_(1.0)
    {
    }

    inline PointRGB3d(const point_t &pos, T a, color_t& c) :
        point_(pos),
        a_(a),
        color_(c)
    {
    }

    virtual ~PointRGB3d() = default;

    inline point_t getPoint() const
    {
        return point_;
    }

    inline float getAlpha() const
    {
        return a_;
    }

    inline color_t getColor() const
    {
        return color_;
    }

    inline void setPoint(const point_t& point)
    {
        point_ = point;
    }

    inline void setAlpha(T a)
    {
        a_ = a;
    }

    inline void getColor(const color_t& c)
    {
        color_ = c;
    }

    inline PointRGB3d min(const PointRGB3d &other) const
    {
        return PointRGB3d(cslibs_math::linear::min(point_, other.point_));
    }

    inline PointRGB3d max(const PointRGB3d &other) const
    {
        return PointRGB3d(cslibs_math::linear::max(point_, other.point_));
    }

    inline static PointRGB3d inf()
    {
        return PointRGB3d(point_t::inf());
    }

    inline static PointRGB3d max()
    {
        return PointRGB3d(point_t::max());
    }

    inline static PointRGB3d min()
    {
        return PointRGB3d(point_t::min());
    }

private:
    point_t point_;
    T       a_;
    color_t color_;
};
}

#endif // CSLIBS_MATH_2D_POINT_3D_HPP
