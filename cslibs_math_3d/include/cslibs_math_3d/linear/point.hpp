#ifndef CSLIBS_MATH_3D_POINT_3D_HPP
#define CSLIBS_MATH_3D_POINT_3D_HPP

#include <cslibs_math/color/color.hpp>
#include <cslibs_math_3d/linear/vector.hpp>

namespace cslibs_math_3d {
using Point3d = Vector3d;

class EIGEN_ALIGN16 PointRGB3d //: public Point3d
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<PointRGB3d>;

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

    inline PointRGB3d(const Point3d &pos) :
        point_(pos),
        a_(1.0)
    {
    }

    inline PointRGB3d(const Point3d &pos, float a, cslibs_math::color::Color& c) :
        point_(pos),
        a_(a),
        color_(c)
    {
    }

    virtual ~PointRGB3d() = default;

    inline Point3d getPoint() const
    {
        return point_;
    }

    inline float getAlpha() const
    {
        return a_;
    }

    inline cslibs_math::color::Color getColor() const
    {
        return color_;
    }

    inline void setPoint(const Point3d& point)
    {
        point_ = point;
    }

    inline void setAlpha(float a)
    {
        a_ = a;
    }

    inline void getColor(const cslibs_math::color::Color& c )
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
        return PointRGB3d(Point3d::inf());
    }

    inline static PointRGB3d max()
    {
        return PointRGB3d(Point3d::max());
    }

    inline static PointRGB3d min()
    {
        return PointRGB3d(Point3d::min());
    }



private:
    Point3d point_;
    float a_;
    cslibs_math::color::Color color_;
};
}

#endif // CSLIBS_MATH_2D_POINT_3D_HPP
