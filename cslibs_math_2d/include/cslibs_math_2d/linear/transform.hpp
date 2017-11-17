#ifndef CSLIBS_MATH_2D_TRANSFORM_2D_HPP
#define CSLIBS_MATH_2D_TRANSFORM_2D_HPP

#include <cslibs_math_2d/linear/vector.hpp>
#include <cslibs_math/common/angle.hpp>

namespace cslibs_math_2d
{
    class Transform2d;
}

inline cslibs_math_2d::Vector2d operator * (const cslibs_math_2d::Transform2d &t,
                                            const cslibs_math_2d::Vector2d &v);

namespace cslibs_math_2d {
class Transform2d {
public:
    inline Transform2d() :
        translation_(0.0, 0.0),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2d(const Vector2d &translation,
                       const double yaw,
                       const double sin,
                       const double cos) :
        translation_(translation),
        yaw_(yaw),
        sin_(sin),
        cos_(cos)
    {
    }

    static inline Transform2d identity()
    {
        return Transform2d(0.0, 0.0);
    }

    inline Transform2d(const double x,
                       const double y) :
        translation_(x, y),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2d(const Vector2d &translation) :
        translation_(translation),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2d(const double yaw) :
        Transform2d(0.0, 0.0, yaw)
    {
    }

    inline Transform2d(const double x,
                       const double y,
                       const double yaw) :
        translation_(x, y),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2d(const Vector2d &translation,
                       const double yaw) :
        translation_(translation),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2d(const Transform2d &other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Transform2d(Transform2d &&other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Transform2d & operator *= (const Transform2d &other)
    {
        if(yaw_ == 0.0) {
            translation_ += other.translation_;
            yaw_ = other.yaw_;
            sin_ = other.sin_;
            cos_ = other.cos_;
        } else if(other.yaw_ == 0.0) {
            translation_ = (*this) * other.translation_;
        } else {
            translation_ = (*this) * other.translation_;
            yaw_ = cslibs_math::common::angle::normalize(yaw_ + other.yaw_);
            const double s = sin_ * other.cos_ + cos_ * other.sin_;
            const double c = cos_ * other.cos_ - sin_ * other.sin_;
            sin_ = s;
            cos_ = c;
        }
        return *this;
    }

    inline Transform2d& operator = (const Transform2d &other)
    {
        yaw_ = other.yaw_;
        sin_ = other.sin_;
        cos_ = other.cos_;
        translation_ = other.translation_;
        return *this;
    }


    inline Transform2d& operator = (const Eigen::Vector3d &eigen)
    {
        translation_ = eigen.block<2,1>(0,0);
        setYaw(eigen(2));
        return *this;
    }

    inline Transform2d& operator = (Transform2d &&other)
    {
        yaw_ = other.yaw_;
        sin_ = other.sin_;
        cos_ = other.cos_;
        translation_ = other.translation_;
        return *this;
    }

    inline Transform2d inverse() const
    {
        Transform2d t;
        t.tx() = -cos_ * translation_(0) - sin_ * translation_(1);
        t.ty() = sin_ * translation_(0) - cos_ * translation_(1);
        t.sin_ = -sin_;
        t.cos_ =  cos_;
        return t;
    }

    inline Transform2d operator -() const
    {
        return inverse();
    }

    inline double & tx()
    {
        return translation_(0);
    }

    inline double tx() const
    {
        return translation_(0);
    }

    inline double & ty()
    {
        return translation_(1);
    }

    inline double ty() const
    {
        return translation_(1);
    }

    inline Vector2d & translation()
    {
        return translation_;
    }

    inline Vector2d const & translation() const
    {
        return translation_;
    }

    inline void setYaw(const double yaw)
    {
        yaw_ = yaw;
        sin_ = std::sin(yaw_);
        cos_ = std::cos(yaw_);
    }

    inline double yaw() const
    {
        return yaw_;
    }

    inline double sin() const
    {
        return sin_;
    }

    inline double cos() const
    {
        return cos_;
    }

    inline Eigen::Vector3d toEigen() const
    {
        return Eigen::Vector3d(translation_(0), translation_(1), yaw_);
    }

    inline void setFrom(const Eigen::Vector3d &eigen)
    {
        translation_(0) = eigen(0);
        translation_(1) = eigen(1);
        setYaw(eigen(2));
    }

    inline void setFrom(const double x,
                        const double y,
                        const double yaw)
    {
        translation_(0) = x;
        translation_(1) = y;
        setYaw(yaw);
    }

    inline Transform2d interpolate(const Transform2d &other,
                                   const double ratio) const
    {
        assert(ratio  >= 0.0);
        assert(ratio <= 1.0);
        if(ratio == 0.0) {
            return *this;
        }
        if(ratio == 1.0) {
            return other;
        }

        const  double ratio_inverse = 1.0 - ratio;
        const  Vector2d translation = translation_ * ratio_inverse + other.translation_ * ratio;
        const  double   yaw = cslibs_math::common::angle::normalize(yaw_ * ratio_inverse + other.yaw_ * ratio);
        return Transform2d(translation, yaw);
    }

private:
    Vector2d translation_;
    double   yaw_;
    double   sin_;
    double   cos_;
} __attribute__ ((aligned (64)));
}

inline cslibs_math_2d::Vector2d operator * (const cslibs_math_2d::Transform2d &t,
                                            const cslibs_math_2d::Vector2d &v)
{
    return t.yaw() == 0.0 ? v + t.translation()
                          : cslibs_math_2d::Vector2d(t.cos() * v(0) - t.sin() * v(1) + t.translation()(0),
                                                     t.sin() * v(0) + t.cos() * v(1) + t.translation()(1));
}

inline cslibs_math_2d::Transform2d operator * (const cslibs_math_2d::Transform2d &a,
                                               const cslibs_math_2d::Transform2d &b)
{
    return a.yaw() == 0.0 ? cslibs_math_2d::Transform2d(b.translation() + a.translation(),
                                                        b.yaw(), b.sin(), b.cos())
                          : b.yaw() == 0.0 ? cslibs_math_2d::Transform2d(a * b.translation(),
                                                                         a.yaw(), a.sin(), a.cos())
                                           : cslibs_math_2d::Transform2d (a * b.translation(),
                                                                          cslibs_math::common::angle::normalize(a.yaw() + b.yaw()),
                                                                          a.sin() * b.cos() + a.cos() * b.sin(),
                                                                          a.cos() * b.cos() - a.sin() * b.sin());
}

inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Transform2d &t)
{
    out << "[" << t.tx() << "," << t.ty() << "," << t.yaw() << "]";
    return out;
}

#endif // CSLIBS_MATH_2D_TRANSFORM_2D_HPP
