#ifndef CSLIBS_MATH_3D_QUATERNION_HPP
#define CSLIBS_MATH_3D_QUATERNION_HPP

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <cslibs_math_3d/linear/vector.hpp>

namespace cslibs_math_3d {
class Quaternion {
public:
    using data_t = double[4];

    inline Quaternion() :
        data_{0.0, 0.0, 0.0, 1.0}
    {
    }

    inline Quaternion(const double yaw) :
        data_{0.0, 0.0, 0.0, 1.0}
    {
        const double yaw_2 = yaw   * 0.5;
        data_[2] = sin(yaw_2);
        data_[3] = cos(yaw_2);
    }

    inline Quaternion(const double roll, const double pitch, const double yaw)
    {
        setRPY(roll, pitch, yaw);
    }

    inline Quaternion(const double x, const double y, const double z, const double w)
    {
        data_[0] = x;
        data_[1] = y;
        data_[2] = z;
        data_[3] = w;
    }

    inline Quaternion(const Quaternion &other)
    {
       assign(other.data_, data_);
    }

    inline Quaternion(Quaternion &&other)
    {
        assign(other.data_, data_);
    }

    inline Quaternion& operator = (const Quaternion &other)
    {
        assign(other.data_, data_);
        return *this;
    }

    inline Quaternion& operator = (Quaternion &&other)
    {
        assign(other.data_, data_);
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

        data_[0] = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
        data_[1] = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
        data_[2] = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
        data_[3] = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
    }

    inline void setRoll(const double roll)
    {
        const double roll_2    = roll  * 0.5;
        data_[0] = std::sin(roll_2);
        data_[1] = 0.0;
        data_[2] = 0.0;
        data_[3] = std::cos(roll_2);
    }

    inline void setPitch(const double pitch)
    {
        const double pitch_2   = pitch * 0.5;
        data_[0] = 0.0;
        data_[1] = std::sin(pitch_2);
        data_[2] = 0.0;
        data_[3] = std::cos(pitch_2);
    }

    inline void setYaw(const double yaw)
    {
        const double yaw_2 = yaw   * 0.5;
        data_[0] = 0.0;
        data_[1] = 0.0;
        data_[2] = std::sin(yaw_2);
        data_[3] = std::cos(yaw_2);
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

    inline Quaternion operator - () const
    {
        Quaternion q;
        q.data_[0] = -data_[0];
        q.data_[1] = -data_[1];
        q.data_[2] = -data_[2];
        q.data_[3] = -data_[3];
        return q;
    }

    inline Quaternion& operator += (const Quaternion &other)
    {
        add(data_, other.data_, data_);
        return *this;
    }


    inline Quaternion& operator -= (const Quaternion &other)
    {
        sub(data_, other.data_, data_);
        return *this;
    }

    inline Quaternion& operator *= (const Quaternion &other)
    {
        data_t q = {data_[0], data_[1], data_[2], data_[3]};
        multiply(q, other.data_, data_);
        return *this;
    }

    inline double angle(const Quaternion &other) const
    {
        return std::acos(dot(data_, other.data_) / std::sqrt(other.norm2() + norm2()));
    }

    inline double angle() const
    {
        return 2.0 * std::acos(data_[3]);
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
        normalize(data_, data_);
    }

    inline Quaternion normalized() const
    {
        Quaternion q;
        normalize(data_, q.data_);
        return q;
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
        Quaternion q;
        invert(data_, q.data_);
        return q;
    }

    inline Quaternion conjugate() const
    {
        Quaternion q;
        conjugate(data_, q.data_);
        return q;
    }

private:
    data_t data_; // [x,y,z,w]

    inline Quaternion(const data_t &data) :
        data_{data[0],data[1],data[2],data[3]}
    {
    }

    inline Quaternion(data_t &&data) :
        data_{data[0],data[1],data[2],data[3]}
    {
    }

    static inline void normalize(const data_t &a, data_t &a_normalized)
    {
        const double n = 1.0 / std::sqrt(dot(a, a));
        a_normalized[0] = a[0] * n;
        a_normalized[1] = a[1] * n;
        a_normalized[2] = a[2] * n;
        a_normalized[3] = a[3] * n;
    }

    static inline void invert(const data_t &a, data_t &a_inverted)
    {
        const double n = 1.0 / dot(a,a);
        a_inverted[0] = -a[0] * n;
        a_inverted[1] = -a[1] * n;
        a_inverted[2] = -a[2] * n;
        a_inverted[3] =  a[3] * n;
    }

    static inline void conjugate(const data_t &q, data_t &q_conjugated)
    {
        q_conjugated[0] = -q[0];
        q_conjugated[1] = -q[1];
        q_conjugated[2] = -q[2];
        q_conjugated[3] =  q[3];
    }

    static inline double dot(const data_t &a, const data_t &b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    }

    static inline void multiply (const data_t &a, const data_t &b, data_t &r)
    {
        r[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
        r[1] = a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2];
        r[2] = a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0];
        r[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    }

    static inline void add(const data_t &a, const data_t &b, data_t &r)
    {
        r[0] = a[0] + b[0];
        r[1] = a[1] + b[1];
        r[2] = a[2] + b[2];
        r[3] = a[3] + b[3];
    }

    static inline void sub(const data_t &a, const data_t &b, data_t &r)
    {
        r[0] = a[0] - b[0];
        r[1] = a[1] - b[1];
        r[2] = a[2] - b[2];
        r[3] = a[3] - b[3];
    }

    static inline void assign(const data_t &a, data_t &b)
    {
        b[0] = a[0];
        b[1] = a[1];
        b[2] = a[2];
        b[3] = a[3];
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


inline cslibs_math_3d::Quaternion operator + (const cslibs_math_3d::Quaternion &a,
                                              const cslibs_math_3d::Quaternion &b)
{
    cslibs_math_3d::Quaternion q;
    q.x() = a.x() + b.x();
    q.y() = a.y() + b.y();
    q.z() = a.z() + b.z();
    q.w() = a.w() + b.w();
    return q;
}

inline cslibs_math_3d::Quaternion operator - (const cslibs_math_3d::Quaternion &a,
                                              const cslibs_math_3d::Quaternion &b)
{
    cslibs_math_3d::Quaternion q;
    q.x() = a.x() - b.x();
    q.y() = a.y() - b.y();
    q.z() = a.z() - b.z();
    q.w() = a.w() - b.w();
    return q;
}

inline cslibs_math_3d::Quaternion operator * (const cslibs_math_3d::Quaternion &a,
                                              const cslibs_math_3d::Quaternion &b)
{
    cslibs_math_3d::Quaternion q;
    q.x() = a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y();
    q.y() = a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z();
    q.z() = a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x();
    q.w() = a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z();
    return q;
}

inline cslibs_math_3d::Vector3d operator * (const cslibs_math_3d::Quaternion &q,
                                            const cslibs_math_3d::Vector3d &v)
{
    cslibs_math_3d::Quaternion inverse = q.invert();
    cslibs_math_3d::Quaternion qv(v(0), v(1), v(2), 0.0);
    qv = q * qv * inverse;
    return cslibs_math_3d::Vector3d(qv.x(),qv.y(),qv.z());
}



#endif // CSLIBS_MATH_3D_QUATERNION_HPP
