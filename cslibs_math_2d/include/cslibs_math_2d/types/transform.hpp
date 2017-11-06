#ifndef CSLIBS_MATH_2D_TRANSFORM_2D_HPP
#define CSLIBS_MATH_2D_TRANSFORM_2D_HPP

#include <cslibs_math_2d/types/vector.hpp>
#include <cslibs_math/common/angle.hpp>

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
        sin_(sin(yaw_)),
        cos_(cos(yaw_))
    {
    }

    inline Transform2d(const Vector2d &translation,
                       const double yaw) :
        translation_(translation),
        yaw_(yaw),
        sin_(sin(yaw_)),
        cos_(cos(yaw_))
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

    inline Vector2d operator * (const Vector2d &v) const
    {
        return yaw_ == 0.0 ? v + translation_
                           : Vector2d(cos_ * v.x() - sin_ * v.y() + translation_.x(),
                                      sin_ * v.x() + cos_ * v.y() + translation_.y());
    }

    inline Transform2d operator * (const Transform2d &other) const
    {
        return yaw_ == 0.0 ? Transform2d(other.translation_ + translation_,
                                         other.yaw_,
                                         other.sin_,
                                         other.cos_)
                           : other.yaw_ == 0.0 ? Transform2d((*this) * other.translation_,
                                                             yaw_,
                                                             sin_,
                                                             cos_)
                                               : Transform2d ((*this) * other.translation_,
                                                              cslibs_math::common::angle::normalize(yaw_ + other.yaw_),
                                                              sin_ * other.cos_ + cos_ * other.sin_,
                                                              cos_ * other.cos_ - sin_ * other.sin_);
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
        translation_.x() = eigen(0);
        translation_.y() = eigen(1);
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
        return Transform2d(Vector2d(-cos_ * translation_.x() - sin_ * translation_.y(),
                                    sin_ * translation_.x() - cos_ * translation_.y()),
                           -yaw_,
                           -sin_,
                           cos_);
    }

    inline Transform2d operator -() const
    {
        return inverse();
    }

    inline double & tx()
    {
        return translation_.x();
    }

    inline double tx() const
    {
        return translation_.x();
    }

    inline double & ty()
    {
        return translation_.y();
    }

    inline double ty() const
    {
        return translation_.y();
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
        return Eigen::Vector3d(translation_.x(), translation_.y(), yaw_);
    }

    inline void setFrom(const Eigen::Vector3d &eigen)
    {
        translation_.x() = eigen(0);
        translation_.y() = eigen(1);
        setYaw(eigen(2));
    }

    inline void setFrom(const double x,
                        const double y,
                        const double yaw)
    {
        translation_.x() = x;
        translation_.y() = y;
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

#endif // CSLIBS_MATH_2D_TRANSFORM_2D_HPP
