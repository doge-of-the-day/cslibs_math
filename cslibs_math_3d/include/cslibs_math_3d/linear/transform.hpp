#ifndef CSLIBS_MATH_3D_TRANSFORM_3D_HPP
#define CSLIBS_MATH_3D_TRANSFORM_3D_HPP

#include <cslibs_math_3d/linear/quaternion.hpp>
#include <cslibs_math_3d/linear/vector.hpp>
#include <cslibs_math/common/angle.hpp>

namespace cslibs_math_3d {
template <typename T>
class EIGEN_ALIGN16 Transform3d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t       = Eigen::aligned_allocator<Transform3d<T>>;

    using eigen_vector_3d_t = Eigen::Matrix<T, 3, 1>;
    using eigen_vector_6d_t = Eigen::Matrix<T, 6, 1>;
    using translation_t     = Vector3d<T>;
    using rotation_t        = Quaternion<T>;

    inline Transform3d()
    {
    }

    static inline Transform3d identity()
    {
        return Transform3d();
    }

    inline Transform3d(const T x,
                       const T y,
                       const T z) :
        translation_(x, y, z)
    {
    }

    inline Transform3d(const translation_t &translation) :
        translation_(translation)
    {
    }

    inline Transform3d(const T yaw) :
        rotation_(yaw)
    {
    }

    inline Transform3d(const T x,
                       const T y,
                       const T z,
                       const T yaw) :
        translation_(x, y, z),
        rotation_(yaw)
    {
    }

    inline Transform3d(const translation_t &translation,
                       const T yaw) :
        translation_(translation),
        rotation_(yaw)
    {
    }

    inline Transform3d(const translation_t &translation,
                       const rotation_t &rotation) :
        translation_(translation),
        rotation_(rotation)
    {
    }

    inline Transform3d(const T x,
                       const T y,
                       const T z,
                       const T roll,
                       const T pitch,
                       const T yaw) :
        translation_(x, y, z),
        rotation_(roll, pitch, yaw)
    {
    }

    inline Transform3d(const Transform3d &other) :
        translation_(other.translation_),
        rotation_(other.rotation_)
    {
    }

    inline Transform3d(Transform3d &&other) :
        translation_(std::move(other.translation_)),
        rotation_(std::move(other.rotation_))
    {
    }

    inline static Transform3d random()
    {
        const Eigen::Matrix<T, 6, 1> r = Eigen::Matrix<T, 6, 1>::Random();
        return Transform3d(r[0],r[1],r[2],
                           r[3],r[4],r[5]);
    }

    inline Transform3d & operator *= (const Transform3d &other)
    {
        translation_ += rotation_ * other.translation_;
        rotation_    *= other.rotation_;
        return *this;
    }

    inline Transform3d& operator = (const Transform3d &other)
    {
        translation_ = other.translation_;
        rotation_ = other.rotation_;
        return *this;
    }


    inline Transform3d& operator = (const Eigen::Matrix<T, 6, 1> &eigen)
    {
        translation_ = translation_t(eigen(0), eigen(1), eigen(2));
        rotation_    = rotation_t(eigen(3), eigen(4), eigen(5));
        return *this;
    }

    inline Transform3d& operator = (Transform3d &&other)
    {
        translation_ = std::move(other.translation_);
        rotation_    = std::move(other.rotation_);
        return *this;
    }

    inline Transform3d inverse() const
    {
        rotation_t rotation_inverse = rotation_.invert();
        return Transform3d(rotation_inverse * -translation_,
                           rotation_inverse);
    }

    inline Transform3d operator -() const
    {
        return inverse();
    }

    inline T & tx()
    {
        return translation_(0);
    }

    inline T tx() const
    {
        return translation_(0);
    }

    inline T & ty()
    {
        return translation_(1);
    }

    inline T ty() const
    {
        return translation_(1);
    }

    inline T& tz()
    {
        return translation_(2);
    }

    inline T tz() const
    {
        return translation_(2);
    }

    inline T roll() const
    {
        return rotation_.roll();
    }

    inline T pitch() const
    {
        return rotation_.pitch();
    }

    inline T yaw() const
    {
        return rotation_.yaw();
    }

    inline void setRoll(const T roll)
    {
        rotation_.setRoll(roll);
    }

    inline void setPitch(const T pitch)
    {
        rotation_.setPitch(pitch);
    }

    inline void setYaw(const T yaw)
    {
        rotation_.setYaw(yaw);
    }

    inline void setRPY(const T roll, const T pitch, const T yaw)
    {
        rotation_.setRPY(roll, pitch, yaw);
    }

    inline translation_t& translation()
    {
        return translation_;
    }

    inline translation_t const& translation() const
    {
        return translation_;
    }

    inline rotation_t& rotation()
    {
        return rotation_;
    }

    inline rotation_t const& rotation() const
    {
        return rotation_;
    }

    inline eigen_vector_3d_t const euler() const
    {
        return cslibs_math::linear::eigen::create<eigen_vector_3d_t>(rotation_.roll(), rotation_.pitch(), rotation_.yaw());
    }

    inline eigen_vector_6d_t toEigen() const
    {
        return cslibs_math::linear::eigen::create<eigen_vector_6d_t>(translation_(0), translation_(1), translation_(2),
                                                                     rotation_.roll(), rotation_.pitch(), rotation_.yaw());
    }

    inline translation_t apply(const translation_t &v) const
    {
        return rotation_ * v + translation_;
    }

    inline void setFrom(const eigen_vector_6d_t &eigen)
    {
        translation_(0) = eigen(0);
        translation_(1) = eigen(1);
        translation_(2) = eigen(2);
        rotation_.setRPY(eigen(3),eigen(4),eigen(5));
    }

    inline void setFrom(const T x,
                        const T y,
                        const T z,
                        const T roll,
                        const T pitch,
                        const T yaw)
    {
        translation_(0) = x;
        translation_(1) = y;
        translation_(2) = z;
        rotation_.setRPY(roll, pitch, yaw);
    }

    inline Transform3d interpolate(const Transform3d &other,
                                   const T ratio) const
    {
        assert(ratio  >= 0.0);
        assert(ratio <= 1.0);
        if(ratio == 0.0) {
            return *this;
        }
        if(ratio == 1.0) {
            return other;
        }

        const  T ratio_inverse = 1.0 - ratio;
        const  translation_t translation = translation_ * ratio_inverse + other.translation_ * ratio;
        const  rotation_t    rotation    = rotation_.interpolate(other.rotation_, ratio);
        return Transform3d(translation, rotation);
    }

private:
    translation_t translation_;
    rotation_t    rotation_;
};
}

template <typename T>
inline cslibs_math_3d::Vector3d<T> operator * (const cslibs_math_3d::Transform3d<T> &t,
                                               const cslibs_math_3d::Vector3d<T>    &v)
{
    return t.rotation() * v + t.translation();
}

template <typename T>
inline Eigen::Matrix<T, 3, 1> operator * (const cslibs_math_3d::Transform3d<T> &t,
                                          const Eigen::Matrix<T, 3, 1>         &v)
{
    return (t.rotation() * cslibs_math_3d::Vector3d<T>(v) + t.translation()).data();
}

template <typename T>
inline cslibs_math_3d::Transform3d<T> operator * (const cslibs_math_3d::Transform3d<T> &a,
                                                  const cslibs_math_3d::Transform3d<T> &b)
{
    return cslibs_math_3d::Transform3d<T>(a.translation() + a.rotation() * b.translation(),
                                       a.rotation() * b.rotation());
}

template <typename T>
inline cslibs_math::linear::Vector<T, 2>  operator * (const cslibs_math_3d::Transform3d<T>    &t,
                                                      const cslibs_math::linear::Vector<T, 2> &v)
{
    const cslibs_math_3d::Vector3d<T> r = t * cslibs_math_3d::Vector3d<T>(v(0), v(1), v(2));
    return cslibs_math::linear::Vector<T, 2> (r(0), r(1));
}

namespace std {
template <typename T>
inline std::string to_string(const cslibs_math_3d::Transform3d<T> &t)
{

    return "[" + std::to_string(t.tx())   + " " + std::to_string(t.ty()) + " " + std::to_string(t.tz()) +
           " " + std::to_string(t.roll()) + " " + std::to_string(t.pitch()) + " " + std::to_string(t.yaw()) +
           "]";
}
}

template <typename T>
inline std::ostream & operator << (std::ostream &out, const cslibs_math_3d::Transform3d<T> &t)
{
    out << "[" << t.tx() << ", " << t.ty() << ", " << t.tz() << ", "
        << t.roll() << ", " << t.pitch() << ", " << t.yaw() << "]";
    return out;
}

#endif // CSLIBS_MATH_3D_TRANSFORM_3D_HPP
