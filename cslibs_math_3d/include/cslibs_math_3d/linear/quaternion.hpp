#ifndef CSLIBS_MATH_3D_QUATERNION_HPP
#define CSLIBS_MATH_3D_QUATERNION_HPP

#include <Eigen/Geometry>
#include <cslibs_math_3d/linear/vector.hpp>

namespace cslibs_math_3d {
class Quaternion {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

    inline double angle(const Quaternion &other) const
    {

    }

    inline Vector3d operator * (const Vector3d &v) const
    {

    }

    inline double roll() const
    {

    }

    inline double pitch() const
    {

    }

    inline double yaw() const
    {

    }

private:
    Eigen::Quaternion<double> data_;
    Eigen::Quaternion<double> data_inverse_;

};

#endif // CSLIBS_MATH_3D_QUATERNION_HPP
