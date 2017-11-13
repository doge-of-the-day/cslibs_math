#ifndef CSLIBS_MATH_3D_TRANSFORM_3D_HPP
#define CSLIBS_MATH_3D_TRANSFORM_3D_HPP

#include <cslibs_math_3d/linear/vector.hpp>
#include <cslibs_math/common/angle.hpp>

namespace cslibs_math_3d {
class Transform3d {
public:
    inline Transform3d() :
        translation_(0.0, 0.0),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    static inline Transform3d identity()
    {
        return Transform3d(0.0, 0.0);
    }

    inline Transform3d(const double x,
                       const double y) :
        translation_(x, y),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform3d(const Vector2d &translation) :
        translation_(translation),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform3d(const double yaw) :
        Transform3d(0.0, 0.0, yaw)
    {
    }

    inline Transform3d(const double x,
                       const double y,
                       const double yaw) :
        translation_(x, y),
        yaw_(yaw),
        sin_(sin(yaw_)),
        cos_(cos(yaw_))
    {
    }

    inline Transform3d(const Vector2d &translation,
                       const double yaw) :
        translation_(translation),
        yaw_(yaw),
        sin_(sin(yaw_)),
        cos_(cos(yaw_))
    {
    }

    inline Transform3d(const Transform3d &other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Transform3d(Transform3d &&other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Vector2d operator * (const Vector2d &v) const
    {
        return yaw_ == 0.0 ? v + translation_
                           : Vector2d(cos_ * v(0) - sin_ * v(1) + translation_(0),
                                      sin_ * v(0) + cos_ * v(1) + translation_(1));
    }

    inline Transform3d operator * (const Transform3d &other) const
    {
        return yaw_ == 0.0 ? Transform3d(other.translation_ + translation_,
                                         other.yaw_,
                                         other.sin_,
                                         other.cos_)
                           : other.yaw_ == 0.0 ? Transform3d((*this) * other.translation_,
                                                             yaw_,
                                                             sin_,
                                                             cos_)
                                               : Transform3d ((*this) * other.translation_,
                                                              cslibs_math::common::angle::normalize(yaw_ + other.yaw_),
                                                              sin_ * other.cos_ + cos_ * other.sin_,
                                                              cos_ * other.cos_ - sin_ * other.sin_);
    }


    inline Transform3d & operator *= (const Transform3d &other)
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

    inline Transform3d& operator = (const Transform3d &other)
    {
        yaw_ = other.yaw_;
        sin_ = other.sin_;
        cos_ = other.cos_;
        translation_ = other.translation_;
        return *this;
    }


    inline Transform3d& operator = (const Eigen::Vector3d &eigen)
    {
        translation_ = eigen.block<2,1>(0,0);
        setYaw(eigen(2));
        return *this;
    }

    inline Transform3d& operator = (Transform3d &&other)
    {
        yaw_ = other.yaw_;
        sin_ = other.sin_;
        cos_ = other.cos_;
        translation_ = other.translation_;
        return *this;
    }

    inline Transform3d inverse() const
    {
        return Transform3d(Vector2d(-cos_ * translation_(0) - sin_ * translation_(1),
                                     sin_ * translation_(0) - cos_ * translation_(1)),
                           -yaw_,
                           -sin_,
                           cos_);
    }

    inline Transform3d operator -() const
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
        sin_ = sin(yaw_);
        cos_ = cos(yaw_);
    }

    inline double yaw() const
    {
        return yaw_;
    }

    inline double sine() const
    {
        return sin_;
    }

    inline double cosine() const
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

    inline Transform3d interpolate(const Transform3d &other,
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
        return Transform3d(translation, yaw);
    }

private:
    inline Transform3d(const Vector2d &translation,
                       const double yaw,
                       const double sin,
                       const double cos) :
        translation_(translation),
        yaw_(yaw),
        sin_(sin),
        cos_(cos)
    {
    }

    Vector2d translation_;
    double   yaw_;
    double   sin_;
    double   cos_;
} __attribute__ ((aligned (64)));
}

inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Transform2d &t)
{
    out << "[" << t.tx() << "," << t.ty() << "," << t.yaw() << "]";
    return out;
}

#endif // CSLIBS_MATH_3D_TRANSFORM_3D_HPP
