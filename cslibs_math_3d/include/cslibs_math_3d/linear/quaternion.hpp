#ifndef CSLIBS_MATH_3D_QUATERNION_HPP
#define CSLIBS_MATH_3D_QUATERNION_HPP

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <cslibs_math_3d/linear/vector.hpp>

namespace cslibs_math_3d {
class Quaternion {
public:
    using data_t = std::array<double, 4>;

    inline Quaternion() :
        data_{{0.0, 0.0, 0.0, 1.0}}
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
        data_{{x,y,z,w}}
    {
    }

    inline Quaternion(const Quaternion &other) :
        data_(other.data_)
    {
    }

    inline Quaternion(Quaternion &&other) :
        data_(std::move(other.data_))
    {
    }

    inline Quaternion& operator = (const Quaternion &other)
    {
        data_         = other.data_;
        return *this;
    }

    inline Quaternion& operator = (Quaternion &&other)
    {
        data_         = std::move(other.data_);
        return *this;
    }

    inline void setRPY(const double roll, const double pitch, const double yaw)
    {
        const double roll_2    = roll  * 0.5;
        const double pitch_2   = pitch * 0.5;
        const double yaw_2     = yaw   * 0.5;
        const double cos_roll  = std::cos(roll_2);
        const double sin_roll  = std::sin(roll_2);
        const double cos_pitch = std::cos(pitch_2);
        const double sin_pitch = std::sin(pitch_2);
        const double cos_yaw   = std::cos(yaw_2);
        const double sin_yaw   = std::sin(yaw_2);

        data_[0] = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw; // x
        data_[1] = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw; // y
        data_[2] = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw; // z
        data_[3] = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw; // w
    }

    inline void setYaw(const double yaw)
    {
        const double yaw_2 = yaw   * 0.5;
        double cos_yaw = std::cos(yaw_2);
        double sin_yaw = std::sin(yaw_2);
        data_[0] = 0.0;
        data_[1] = 0.0;
        data_[2] = sin_yaw;
        data_[3] = cos_yaw;
    }

    inline double & w()
    {
        return data_[3];
    }

    inline double & x()
    {
        return data_[0];
    }

    inline double & y()
    {
        return data_[1];
    }

    inline double & z()
    {
        return data_[2];
    }

    inline double w() const
    {
        return data_[3];
    }

    inline double x() const
    {
        return data_[0];
    }

    inline double y() const
    {
        return data_[1];
    }

    inline double z() const
    {
        return data_[2];

    }

    inline Quaternion operator + (const Quaternion &other) const
    {
        return Quaternion(data_[0] + other.data_[0],
                          data_[1] + other.data_[1],
                          data_[2] + other.data_[2],
                          data_[3] + other.data_[3]);
    }


    inline Quaternion& operator += (const Quaternion &other)
    {
        data_[0] += other.data_[0];
        data_[1] += other.data_[1];
        data_[2] += other.data_[2];
        data_[3] += other.data_[3];
        return *this;
    }

    inline Quaternion operator - (const Quaternion &other) const
    {
        return Quaternion(data_[0] - other.data_[0],
                data_[1] - other.data_[1],
                data_[2] - other.data_[2],
                data_[3] - other.data_[3]);
    }

    inline Quaternion& operator -= (const Quaternion &other)
    {
        data_[0] -= other.data_[0];
        data_[1] -= other.data_[1];
        data_[2] -= other.data_[2];
        data_[3] -= other.data_[3];
        return *this;
    }

    inline Quaternion operator - () const
    {
        return Quaternion(-data_[0],
                          -data_[1],
                          -data_[2],
                          -data_[3]);
    }

    inline Quaternion operator * (const Quaternion &other) const
    {
        return Quaternion(multiply(data_, other.data_));
    }

    inline Quaternion& operator *= (const Quaternion &other)
    {
        data_ = multiply(data_, other.data_);
        return *this;
    }

    inline double angle(const Quaternion &other) const
    {
        const double s = std::sqrt(other.norm2() + norm2());
        return std::acos(dot(data_, other.data_) / s);
    }

    inline double angle() const
    {
        return 2.0 * std::acos(data_[3]);
    }

    inline Vector3d operator * (const Vector3d &v) const
    {
        const data_t rot = multiply(data_, multiply({{v(0), v(1), v(2), 0.0}}, invert(data_)));
        return Vector3d(rot[0],rot[1],rot[2]);
    }

    inline double roll() const
    {
        const double sin_roll = 2.0 * (data_[3] * data_[0] + data_[1] * data_[2]);
        const double cos_roll = 1.0 - 2.0 * (data_[0] * data_[0] + data_[1] * data_[1]);
        return std::atan2(sin_roll, cos_roll);
    }

    inline double pitch() const
    {
        const double sin_pitch = 2.0 * (data_[3]  * data_[1] - data_[2]  * data_[0] );
        return std::abs(sin_pitch) >= 1.0 ? std::copysign(M_PI / 2, sin_pitch) : std::asin(sin_pitch);
    }

    inline double yaw() const
    {
        const double sin_yaw = 2.0 * (data_[3]  * data_[2]  + data_[0]  * data_[1] );
        const double cos_yaw = 1.0 - 2.0 * (data_[1]  * data_[1]  + data_[2] * data_[2]);
        return std::atan2(sin_yaw, cos_yaw);
    }

    inline void getRollPitchYaw(double &r, double &p, double &y) const
    {
        r = roll();
        p = pitch();
        y = yaw();
    }

    inline void normalize()
    {
        data_ = normalize(data_);
    }

    inline Quaternion normalized() const
    {
        return Quaternion(normalize(data_));
    }

    inline double norm() const
    {
        return std::sqrt(norm2());
    }

    inline double norm2() const
    {
        return dot(data_, data_);
    }

    inline Quaternion interpolate(const Quaternion &other,
                                  const double ratio) const
    {
        assert(ratio >= 0.0 && ratio <= 1.0);
        const double s = std::sqrt(norm2()* other.norm2());
        assert(s != 0.0);
        const double d = dot(data_, other.data_);
        const double theta = d < 0.0 ? std::acos(dot(data_, (-other).data_) / s) : std::acos(d / s);
        if(theta != 0.0) {
            const double scale = 1.0 / std::sin(theta);
            const double s0 = std::sin((1.0 - ratio) * theta);
            const double s1 = std::sin(ratio * theta);
            return d < 0.0 ? Quaternion((data_[0] * s0 - other.data_[0] * s1) * scale,
                                        (data_[1] * s0 - other.data_[1] * s1) * scale,
                                        (data_[2] * s0 - other.data_[2] * s1) * scale,
                                        (data_[3] * s0 - other.data_[3] * s1) * scale)
                           : Quaternion((data_[0] * s0 + other.data_[0] * s1) * scale,
                                        (data_[1] * s0 + other.data_[1] * s1) * scale,
                                        (data_[2] * s0 + other.data_[2] * s1) * scale,
                                        (data_[3] * s0 + other.data_[3] * s1) * scale);
        }
        return *this;
    }

    inline Quaternion invert() const
    {
        return Quaternion(invert(data_));
    }

    inline Quaternion conjugate() const
    {
        return Quaternion(conjugate(data_));
    }

private:
    data_t data_;

    inline Quaternion(const std::array<double, 4> &data) :
        data_(data)
    {
    }

    inline Quaternion(std::array<double, 4> &&data) :
        data_(data)
    {
    }

    static inline std::array<double, 4> normalize(const std::array<double, 4> &a)
    {
        const double n = 1.0 / std::sqrt(dot(a, a));
        return {{a[0] * n, a[1] * n, a[2] * n, a[3] * n}};
    }

    static inline std::array<double, 4> invert(const double x, const double y, const double z, const double w)
    {
        const double n = 1.0 / (x*x + y*y + z*z + w*w);
        return {{-x * n, -y * n, -z * n, w * n}};
    }

    static inline std::array<double, 4> invert(const std::array<double, 4> &a)
    {
        const double n = 1.0 / dot(a,a);
        return {{-a[0] * n, -a[1] * n, -a[2] * n, a[3] * n}};
    }

    static inline std::array<double,4> conjugate(const std::array<double, 4> &q)
    {
        return {{-q[0], -q[1], -q[2], q[3]}};
    }

    static inline double dot(const std::array<double,4> &a, const std::array<double,4> &b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    }

    static inline std::array<double, 4> multiply (const std::array<double, 4> &a, const std::array<double,4> &b)
    {
        return {{a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
                        a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2],
                        a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0],
                        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]}};
    }

    inline double angleShortestPath(const Quaternion& q) const
    {
        const double s = std::sqrt(norm2()* q.norm2());
        assert(s != 0.0);
        const double d = dot(data_, q.data_);
        return d < 0.0 ? std::acos(dot(data_, (-q).data_) / s) : std::acos(d / s);
    }

}__attribute__ ((aligned (32)));
}



#endif // CSLIBS_MATH_3D_QUATERNION_HPP
