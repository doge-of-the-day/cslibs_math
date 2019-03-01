#ifndef CSLIBS_MATH_2D_TRANSFORM_2D_HPP
#define CSLIBS_MATH_2D_TRANSFORM_2D_HPP

#include <cslibs_math_2d/linear/vector.hpp>
#include <cslibs_math/common/angle.hpp>

namespace cslibs_math_2d
{
    template <typename T>
    class Transform2d;
}

template <typename T>
inline cslibs_math_2d::Vector2d<T> operator * (const cslibs_math_2d::Transform2d<T> &t,
                                               const cslibs_math_2d::Vector2d<T> &v);

namespace cslibs_math_2d {
template <typename T>
class EIGEN_ALIGN16 Transform2d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Transform2d<T>>;

    inline Transform2d() :
        translation_(0.0, 0.0),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2d(const Vector2d<T> &translation,
                       const T            yaw,
                       const T            sin,
                       const T            cos) :
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

    inline Transform2d(const T x,
                       const T y) :
        translation_(x, y),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2d(const Vector2d<T> &translation) :
        translation_(translation),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2d(const T yaw) :
        Transform2d(0.0, 0.0, yaw)
    {
    }

    inline Transform2d(const T x,
                       const T y,
                       const T yaw) :
        translation_(x, y),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2d(const Vector2d<T> &translation,
                       const T yaw) :
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
        translation_(std::move(other.translation_)),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline static Transform2d random()
    {
        const Eigen::Matrix<T,3,1> r = Eigen::Matrix<T,3,1>::Random();
        return Transform2d(r(0),r(1),r(2));
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
            const T s = sin_ * other.cos_ + cos_ * other.sin_;
            const T c = cos_ * other.cos_ - sin_ * other.sin_;
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


    inline Transform2d& operator = (const Eigen::Matrix<T,3,1> &eigen)
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
        return Transform2d(Vector2d<T>(-(cos_ *  translation_(0) + sin_ * translation_(1)),
                                       -(-sin_ * translation_(0) + cos_ * translation_(1))),
                           -yaw_,
                           -sin_,
                           cos_);
    }

    inline Transform2d operator -() const
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

    inline Vector2d<T> & translation()
    {
        return translation_;
    }

    inline Vector2d<T> const & translation() const
    {
        return translation_;
    }

    inline void setYaw(const T yaw)
    {
        yaw_ = yaw;
        sin_ = std::sin(yaw_);
        cos_ = std::cos(yaw_);
    }

    inline T yaw() const
    {
        return yaw_;
    }

    inline T sin() const
    {
        return sin_;
    }

    inline T cos() const
    {
        return cos_;
    }

    inline Eigen::Matrix<T,3,1> toEigen() const
    {
        return Eigen::Matrix<T,3,1>(translation_(0), translation_(1), yaw_);
    }

    inline Eigen::Matrix<T,2,2> getEigenRotation() const
    {
        Eigen::Matrix<T,2,2> rot;
        rot(0,0) =  cos_;
        rot(0,1) = -sin_;
        rot(1,0) =  sin_;
        rot(1,1) =  cos_;
        return rot;
    }

    inline void setFrom(const Eigen::Matrix<T,3,1> &eigen)
    {
        translation_(0) = eigen(0);
        translation_(1) = eigen(1);
        setYaw(eigen(2));
    }

    inline void setFrom(const T x,
                        const T y,
                        const T yaw)
    {
        translation_(0) = x;
        translation_(1) = y;
        setYaw(yaw);
    }

    inline Transform2d interpolate(const Transform2d &other,
                                   const T ratio) const
    {
        assert(ratio >= 0.0);
        assert(ratio <= 1.0);
        if (ratio == 0.0)
            return *this;

        if (ratio == 1.0)
            return other;

        const T ratio_inverse = 1.0 - ratio;
        const Vector2d<T> translation = translation_ * ratio_inverse + other.translation_ * ratio;
        const T yaw = cslibs_math::common::angle::normalize(yaw_ + ratio * cslibs_math::common::angle::normalize(other.yaw_ - yaw_));
        return Transform2d(translation, yaw);
    }

    inline std::string toString() const
    {
        return "[" + std::to_string(tx()) + "," +
                     std::to_string(ty()) + "," +
                     std::to_string(yaw()) + "]";
    }

private:
    Vector2d<T> translation_;
    T           yaw_;
    T           sin_;
    T           cos_;
};
}

template <typename T>
inline cslibs_math_2d::Vector2d<T> operator * (const cslibs_math_2d::Transform2d<T> &t,
                                               const cslibs_math_2d::Vector2d<T> &v)
{
    return t.yaw() == 0.0 ? v + t.translation()
                          : cslibs_math_2d::Vector2d<T>(t.cos() * v(0) - t.sin() * v(1) + t.translation()(0),
                                                        t.sin() * v(0) + t.cos() * v(1) + t.translation()(1));
}

template <typename T>
inline cslibs_math_2d::Transform2d<T> operator * (const cslibs_math_2d::Transform2d<T> &a,
                                                  const cslibs_math_2d::Transform2d<T> &b)
{
    return a.yaw() == 0.0 ? cslibs_math_2d::Transform2d<T>(b.translation() + a.translation(),
                                                           b.yaw(), b.sin(), b.cos())
                          : b.yaw() == 0.0 ? cslibs_math_2d::Transform2d<T>(a * b.translation(),
                                                                            a.yaw(), a.sin(), a.cos())
                                           : cslibs_math_2d::Transform2d<T>(a * b.translation(),
                                                                            cslibs_math::common::angle::normalize(a.yaw() + b.yaw()),
                                                                            a.sin() * b.cos() + a.cos() * b.sin(),
                                                                            a.cos() * b.cos() - a.sin() * b.sin());
}

template <typename T>
inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Transform2d<T> &t)
{
    out << "[" << t.tx() << "," << t.ty() << "," << t.yaw() << "]";
    return out;
}

#endif // CSLIBS_MATH_2D_TRANSFORM_2D_HPP
