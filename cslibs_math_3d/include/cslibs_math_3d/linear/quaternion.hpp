#ifndef CSLIBS_MATH_3D_QUATERNION_HPP
#define CSLIBS_MATH_3D_QUATERNION_HPP

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <cslibs_math_3d/linear/vector.hpp>

namespace cslibs_math_3d {
class Quaternion {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using quaternion_t = Eigen::Quaternion<double>;

    inline Quaternion() :
        data_(1.0, 0.0, 0.0, 0.0),
        data_inverse_(1.0, 0.0, 0.0, 0.0)
    {
    }

    inline Quaternion(const double yaw)
    {
        setYaw(yaw);
    }

    inline Quaternion(const double roll, const double pitch, const double yaw)
    {
        setRPY(roll, pitch, yaw);
    }

    inline Quaternion(const double x, const double y, const double z, const double w) :
        data_(x,y,z,w),
        data_inverse_(data_.inverse())
    {
    }

    inline Quaternion(const Quaternion &other) :
        data_(other.data_),
        data_inverse_(other.data_inverse_)
    {
    }

    inline Quaternion(Quaternion &&other) :
        data_(std::move(other.data_)),
        data_inverse_(std::move(other.data_inverse_))
    {
    }

    inline Quaternion& operator = (const Quaternion &other)
    {
        data_ = other.data_;
        data_inverse_ = other.data_inverse_;
        return *this;
    }

    inline Quaternion& operator = (const Quaternion &&other)
    {
        data_ = std::move(other.data_);
        data_inverse_ = std::move(other.data_inverse_);
        return *this;
    }

    inline void setRPY(const double roll, const double pitch, const double yaw)
    {
        const double roll_2  = roll  * 0.5;
        const double pitch_2 = pitch * 0.5;
        const double yaw_2   = yaw   * 0.5;
        double cos_roll, sin_roll;
        sincos(roll_2, &sin_roll, &cos_roll);
        double cos_pitch, sin_pitch;
        sincos(pitch_2, &sin_pitch, &cos_pitch);
        double cos_yaw, sin_yaw;
        sincos(yaw_2, &sin_yaw, &cos_yaw);
        auto &coeffs = data_.coeffs();
        coeffs(0) =  sin_roll * cos_yaw - sin_pitch *  sin_yaw; // x
        coeffs(1) = sin_pitch *  cos_yaw +  sin_roll * sin_yaw; // y
        coeffs(2) =   sin_yaw - sin_pitch * sin_roll * cos_yaw; // z
        coeffs(3) =   cos_yaw + sin_pitch * sin_roll * sin_yaw; // w
        data_inverse_ = data_.inverse();
    }

    inline void setYaw(const double yaw)
    {
        const double yaw_2 = yaw   * 0.5;
        double cos_yaw, sin_yaw;
        sincos(yaw_2, &sin_yaw, &cos_yaw);
        auto &coeffs = data_.coeffs();
        coeffs(0) = 0.0; // x
        coeffs(1) = 0.0; // y
        coeffs(2) = sin_yaw; // z
        coeffs(3) = cos_yaw; // w
        data_inverse_ = data_.inverse();
    }

    inline double & w()
    {
        return data_.coeffs()(3);
    }

    inline double & x()
    {
        return data_.coeffs()(0);
    }

    inline double & y()
    {
        return data_.coeffs()(1);
    }

    inline double & z()
    {
        return data_.coeffs()(2);
    }

    inline double w() const
    {
        return data_.coeffs()(3);
    }

    inline double x() const
    {
        return data_.coeffs()(0);
    }

    inline double y() const
    {
        return data_.coeffs()(1);
    }

    inline double z() const
    {
        return data_.coeffs()(2);
    }

    inline Quaternion operator + (const Quaternion &other) const
    {
        return Quaternion(data_.coeffs() + other.data_.coeffs());
    }

    inline Quaternion& operator += (const Quaternion &other)
    {
        data_.coeffs() += other.data_.coeffs();
        data_inverse_   = data_.inverse();
        return *this;
    }

    inline Quaternion operator - (const Quaternion &other) const
    {
        return Quaternion(data_.coeffs() - other.data_.coeffs());
    }

    inline Quaternion& operator -= (const Quaternion &other)
    {
        data_.coeffs() -= other.data_.coeffs();
        data_inverse_   = data_.inverse();
        return *this;
    }

    inline Quaternion operator - () const
    {
        return Quaternion(-data_.coeffs());
    }

    inline double angle(const Quaternion &other) const
    {
        const double s = std::sqrt(other.norm2() + norm2());
        return std::acos(data_.dot(other.data_) / s);
    }

    inline double angle() const
    {
        return 2.0 * std::acos(w());
    }

    inline Vector3d operator * (const Vector3d &v) const
    {
        auto q = [](const Vector3d &v) { return quaternion_t (0.0, v(0), v(1), v(2)); };

        const quaternion_t rot = data_ * q(v) * data_inverse_;
        return Vector3d(Eigen::Vector3d(rot.vec()));
    }

    inline double roll() const
    {
        const auto &coeffs = data_.coeffs();
        const double sin_roll = 2.0 * coeffs(0) * coeffs(3) + coeffs(1) * coeffs(2);
        const double cos_roll = 1.0 - 2.0 * (coeffs(0) * coeffs(0) + coeffs(1) * coeffs(1));
        return std::atan2(sin_roll, cos_roll);
    }

    inline double pitch() const
    {
        const auto &coeffs = data_.coeffs();
        const double sin_pitch = 2.0 * coeffs(1) * coeffs(3) - coeffs(0) * coeffs(2);
        return std::abs(sin_pitch) >= 1.0 ? std::copysign(M_PI / 2, sin_pitch) : std::asin(sin_pitch);
    }

    inline double yaw() const
    {
        const auto &coeffs = data_.coeffs();
        const double sin_yaw = 2.0 * coeffs(2) * coeffs(3) + coeffs(0) * coeffs(1);
        const double cos_yaw = 1.0 - 2.0 * (coeffs(1) * coeffs(1) + coeffs(2) * coeffs(2));
        return std::atan2(sin_yaw, cos_yaw);
    }

    inline void getRollPitchYaw(double &roll, double &pitch, double &yaw) const
    {
        const auto &coeffs = data_.coeffs();
        const double sin_roll = 2.0 * coeffs(0) * coeffs(3) + coeffs(1) * coeffs(2);
        const double cos_roll = 1.0 - 2.0 * (coeffs(0) * coeffs(0) + coeffs(1) * coeffs(1));
        roll = std::atan2(sin_roll, cos_roll);
        const double sin_pitch = 2.0 * coeffs(1) * coeffs(3) - coeffs(0) * coeffs(2);
        pitch = std::abs(sin_pitch) >= 1.0 ? std::copysign(M_PI / 2, sin_pitch) : std::asin(sin_pitch);
        const double sin_yaw = 2.0 * coeffs(2) * coeffs(3) + coeffs(0) * coeffs(1);
        const double cos_yaw = 1.0 - 2.0 * (coeffs(1) * coeffs(1) + coeffs(2) * coeffs(2));
        yaw = std::atan2(sin_yaw, cos_yaw);
    }

    inline void normalize()
    {
        data_.normalize();
        data_inverse_ = data_.inverse();
    }

    inline Quaternion normalized() const
    {
        return Quaternion(data_.normalized());
    }

    inline double norm() const
    {
        return data_.norm();
    }

    inline double norm2() const
    {
        return data_.squaredNorm();
    }

    inline Quaternion interpolate(const Quaternion &other,
                                  const double ratio) const
    {
        assert(ratio >= 0.0 && ratio <= 1.0);
        return Quaternion(data_.slerp(ratio, other.data_));
    }

    inline Quaternion inverse() const
    {
        return Quaternion(data_inverse_, data_);
    }

    inline Quaternion conjugate() const
    {
        const quaternion_t con = data_.conjugate();
        return Quaternion(con, con);
    }

    static inline Quaternion random()
    {
        return Quaternion(quaternion_t::Coefficients::Random()).normalized();
    }

private:
    quaternion_t data_;
    quaternion_t data_inverse_;

    inline Quaternion(const quaternion_t::Coefficients &coeffs) :
        data_(coeffs(0),coeffs(1),coeffs(2),coeffs(3)),
        data_inverse_(data_.inverse())
    {
    }

    inline Quaternion(const quaternion_t &data) :
        data_(data),
        data_inverse_(data.inverse())
    {
    }

    inline Quaternion(const quaternion_t &data,
                      const quaternion_t &data_inverse) :
        data_(data),
        data_inverse_(data_inverse)
    {
    }
};
}

#endif // CSLIBS_MATH_3D_QUATERNION_HPP
