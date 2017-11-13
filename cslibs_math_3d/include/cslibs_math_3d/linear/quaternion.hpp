#ifndef CSLIBS_MATH_3D_QUATERNION_HPP
#define CSLIBS_MATH_3D_QUATERNION_HPP

#include <Eigen/Geometry>
#include <cslibs_math_3d/linear/vector.hpp>

namespace cslibs_math_3d {
class Quaternion {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using quaternion_t = Eigen::Quaternion<double>;

    Quaternion() :
        data_(1.0, 0.0, 0.0, 0.0),
        data_inverse_(1.0, 0.0, 0.0, 0.0)
    {
    }

    Quaternion(const double yaw)
    {

    }

    Quaternion(const double roll, const double pitch, const double yaw)
    {

    }

    Quaternion(const double x, const double y, const double z, const double w)
    {
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
        data_inverse_ = data_.inverse()
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
        data_inverse_ = data_.inverse()
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

    inline Quaternion operator * (const Quaternion &other) const
    {

    }

    inline Quaternion& operator *= (const Quaternion &other)
    {

        return *this;
    }


    inline double angle(const Quaternion &other) const
    {

    }

    inline Vector3d operator * (const Vector3d &v) const
    {
        const quaternion_t rot = data_ * v * data_inverse_;
        return Vector3d(rot.vec());
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
        const double sin_pitch = 2.0 * coeffs(1) * coeffs(3) - coeffs(0) * cosffs(2);
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
        const double sin_pitch = 2.0 * coeffs(1) * coeffs(3) - coeffs(0) * cosffs(2);
        pitch = std::abs(sin_pitch) >= 1.0 ? std::copysign(M_PI / 2, sin_pitch) : std::asin(sin_pitch);
        const double sin_yaw = 2.0 * coeffs(2) * coeffs(3) + coeffs(0) * coeffs(1);
        const double cos_yaw = 1.0 - 2.0 * (coeffs(1) * coeffs(1) + coeffs(2) * coeffs(2));
        yaw = std::atan2(sin_yaw, cos_yaw);
    }

private:
    quaternion_t data_;
    quaternion_t data_inverse_;

};

#endif // CSLIBS_MATH_3D_QUATERNION_HPP
