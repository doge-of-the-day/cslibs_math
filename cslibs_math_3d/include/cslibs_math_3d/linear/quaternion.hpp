#ifndef CSLIBS_MATH_3D_QUATERNION_HPP
#define CSLIBS_MATH_3D_QUATERNION_HPP

#include <array>
#include <cslibs_math_3d/linear/vector.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace cslibs_math_3d {
template <typename T>
class EIGEN_ALIGN16 Quaternion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using data_t = std::array<T, 4>;

  inline Quaternion() = default;

  inline explicit Quaternion(const T yaw) {
    const T yaw_2 = yaw * 0.5;
    data_[2] = sin(yaw_2);
    data_[3] = cos(yaw_2);
  }

  inline explicit Quaternion(const T roll, const T pitch, const T yaw) {
    setRPY(roll, pitch, yaw);
  }

  inline explicit Quaternion(const T x, const T y, const T z, const T w)
      : data_{{x, y, z, w}} {}

  inline explicit Quaternion(const T angle, const Vector3<T> &axis) {
    T sa = std::sin(0.5 * angle);
    T norm = axis.length();
    data_[0] = sa * axis(0) / norm;
    data_[1] = sa * axis(1) / norm;
    data_[2] = sa * axis(2) / norm;
    data_[3] = std::cos(0.5 * angle);
  }

  inline Quaternion(const Quaternion &other) = default;
  inline Quaternion(Quaternion &&other) = default;
  inline Quaternion &operator=(const Quaternion &other) = default;
  inline Quaternion &operator=(Quaternion &&other) = default;
  ;

  inline static Quaternion fromAngleAxis(const Vector3<T> &angle_axis) {
    auto scale = 0.5;
    auto w = 1.0;
    if (angle_axis.length2() > 1e-8) {
      const auto norm = angle_axis.length();
      scale = std::sin(norm * 0.5) / norm;
      w = std::cos(norm * 0.5);
    }

    const auto xyz = angle_axis * scale;
    return {xyz(0), xyz(1), xyz(2), w};
  }

  inline static Quaternion fromEigen(const Eigen::Quaternion<T> &eigen) {
    return Quaternion{eigen.x(), eigen.y(), eigen.z(), eigen.w()};
  }

  inline void setRPY(const T roll, const T pitch, const T yaw) {
    const T roll_2 = roll * 0.5;
    const T pitch_2 = pitch * 0.5;
    const T yaw_2 = yaw * 0.5;
    const T cos_roll = std::cos(roll_2);
    const T sin_roll = std::sin(roll_2);
    const T cos_pitch = std::cos(pitch_2);
    const T sin_pitch = std::sin(pitch_2);
    const T cos_yaw = std::cos(yaw_2);
    const T sin_yaw = std::sin(yaw_2);

    data_[0] = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw;
    data_[1] = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw;
    data_[2] = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw;
    data_[3] = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw;
  }

  inline void setRoll(const T roll) {
    const T roll_2 = roll * 0.5;
    data_[0] = std::sin(roll_2);
    data_[1] = T();
    data_[2] = T();
    data_[3] = std::cos(roll_2);
  }

  inline void setPitch(const T pitch) {
    const T pitch_2 = pitch * 0.5;
    data_[0] = T();
    data_[1] = std::sin(pitch_2);
    data_[2] = T();
    data_[3] = std::cos(pitch_2);
  }

  inline void setYaw(const T yaw) {
    const T yaw_2 = yaw * 0.5;
    data_[0] = T();
    data_[1] = T();
    data_[2] = std::sin(yaw_2);
    data_[3] = std::cos(yaw_2);
  }

  inline T &w() { return data_[3]; }

  inline T &x() { return data_[0]; }

  inline T &y() { return data_[1]; }

  inline T &z() { return data_[2]; }

  inline T w() const { return data_[3]; }

  inline T x() const { return data_[0]; }

  inline T y() const { return data_[1]; }

  inline T z() const { return data_[2]; }

  inline Quaternion operator-() const {
    Quaternion q;
    q.data_[0] = -data_[0];
    q.data_[1] = -data_[1];
    q.data_[2] = -data_[2];
    q.data_[3] = -data_[3];
    return q;
  }

  inline Quaternion &operator+=(const Quaternion &other) {
    add(data_, other.data_, data_);
    return *this;
  }

  inline Quaternion &operator-=(const Quaternion &other) {
    sub(data_, other.data_, data_);
    return *this;
  }

  inline Quaternion &operator*=(const Quaternion &other) {
    data_t q = {data_[0], data_[1], data_[2], data_[3]};
    multiply(q, other.data_, data_);
    return *this;
  }

  inline T angle(const Quaternion &other) const {
    return std::acos(dot(data_, other.data_) /
                     std::sqrt(other.norm2() + norm2()));
  }

  inline T angle() const { return 2.0 * std::acos(data_[3]); }

  inline T roll() const {
    const T sin_roll = 2.0 * (data_[3] * data_[0] + data_[1] * data_[2]);
    const T cos_roll = 1.0 - 2.0 * (data_[0] * data_[0] + data_[1] * data_[1]);
    return std::atan2(sin_roll, cos_roll);
  }

  inline T pitch() const {
    const T sin_pitch = 2.0 * (data_[3] * data_[1] - data_[2] * data_[0]);
    return std::abs(sin_pitch) >= 1 ? std::copysign(M_PI / 2.0, sin_pitch)
                                    : std::asin(sin_pitch);
  }

  inline T yaw() const {
    const T sin_yaw = 2.0 * (data_[3] * data_[2] + data_[0] * data_[1]);
    const T cos_yaw = 1.0 - 2.0 * (data_[1] * data_[1] + data_[2] * data_[2]);
    return std::atan2(sin_yaw, cos_yaw);
  }

  inline void getRollPitchYaw(T &r, T &p, T &y) const {
    r = roll();
    p = pitch();
    y = yaw();
  }

  inline void normalize() { normalize(data_, data_); }

  inline Quaternion normalized() const {
    Quaternion q;
    normalize(data_, q.data_);
    return q;
  }

  inline T norm() const { return std::sqrt(norm2()); }

  inline T norm2() const { return dot(data_, data_); }

  inline Quaternion interpolate(const Quaternion &other, const T ratio) const {
    assert(ratio >= 0.0 && ratio <= 1.0);
    const T s = std::sqrt(norm2() * other.norm2());
    assert(s != 0.0);
    const T d = dot(data_, other.data_);
    const T theta =
        d < 0 ? std::acos(dot(data_, (-other).data_) / s) : std::acos(d / s);
    if (theta != T()) {
      const T scale = 1.0 / std::sin(theta);
      const T s0 = std::sin((1.0 - ratio) * theta);
      const T s1 = std::sin(ratio * theta);
      return d < 0 ? Quaternion((data_[0] * s0 - other.data_[0] * s1) * scale,
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

  inline Quaternion invert() const {
    Quaternion q;
    invert(data_, q.data_);
    return q;
  }

  inline Quaternion conjugate() const {
    Quaternion q;
    conjugate(data_, q.data_);
    return q;
  }

  Vector3<T> toAngleAxis() const {
    auto normed = normalized();
    if (normed.w() < 0) normed = -normed;
    const auto xyz = Vector3<T>(normed.x(), normed.y(), normed.z());

    const auto angle = 2.0 * std::atan2(xyz.length(), normed.w());
    const auto scale = angle < 1e-7 ? 2.0 : angle / std::sin(angle * 0.5);
    return xyz * scale;
  }

  Eigen::Quaternion<T> toEigen() const {
    return Eigen::Quaternion<T>{w(), x(), y(), z()};
  }

  inline static Quaternion random() {
    const auto angles = Vector3<T>::random();
    Quaternion q;
    q.setRPY(angles(0), angles(1), angles(2));
    return q;
  }

 private:
  data_t data_{{T{0}, T{0}, T{0}, T{1}}};  // [x,y,z,w]

  inline Quaternion(const data_t &data)
      : data_{{data[0], data[1], data[2], data[3]}} {}

  inline Quaternion(data_t &&data)
      : data_{{data[0], data[1], data[2], data[3]}} {}

  inline static void normalize(const data_t &a, data_t &a_normalized) {
    const T n = 1.0 / std::sqrt(dot(a, a));
    a_normalized[0] = a[0] * n;
    a_normalized[1] = a[1] * n;
    a_normalized[2] = a[2] * n;
    a_normalized[3] = a[3] * n;
  }

  inline static void invert(const data_t &a, data_t &a_inverted) {
    const T n = 1.0 / dot(a, a);
    a_inverted[0] = -a[0] * n;
    a_inverted[1] = -a[1] * n;
    a_inverted[2] = -a[2] * n;
    a_inverted[3] = a[3] * n;
  }

  inline static void conjugate(const data_t &q, data_t &q_conjugated) {
    q_conjugated[0] = -q[0];
    q_conjugated[1] = -q[1];
    q_conjugated[2] = -q[2];
    q_conjugated[3] = q[3];
  }

  inline static T dot(const data_t &a, const data_t &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  }

  inline static void multiply(const data_t &a, const data_t &b, data_t &r) {
    r[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    r[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    r[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    r[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    /*
    r[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    r[1] = a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2];
    r[2] = a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0];
    r[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];*/
  }

  inline static void add(const data_t &a, const data_t &b, data_t &r) {
    r[0] = a[0] + b[0];
    r[1] = a[1] + b[1];
    r[2] = a[2] + b[2];
    r[3] = a[3] + b[3];
  }

  inline static void sub(const data_t &a, const data_t &b, data_t &r) {
    r[0] = a[0] - b[0];
    r[1] = a[1] - b[1];
    r[2] = a[2] - b[2];
    r[3] = a[3] - b[3];
  }

  inline static void assign(const data_t &a, data_t &b) {
    b[0] = a[0];
    b[1] = a[1];
    b[2] = a[2];
    b[3] = a[3];
  }

  inline T angleShortestPath(const Quaternion &q) const {
    const T s = std::sqrt(norm2() * q.norm2());
    assert(s != 0.0);
    const T d = dot(data_, q.data_);
    return d < 0 ? std::acos(dot(data_, (-q).data_) / s) : std::acos(d / s);
  }
};

using Quaterniond = Quaternion<double>;
using Quaternionf = Quaternion<float>;
}  // namespace cslibs_math_3d

template <typename T>
inline cslibs_math_3d::Quaternion<T> operator+(
    const cslibs_math_3d::Quaternion<T> &a,
    const cslibs_math_3d::Quaternion<T> &b) {
  cslibs_math_3d::Quaternion<T> q;
  q.x() = a.x() + b.x();
  q.y() = a.y() + b.y();
  q.z() = a.z() + b.z();
  q.w() = a.w() + b.w();
  return q;
}

template <typename T>
inline cslibs_math_3d::Quaternion<T> operator-(
    const cslibs_math_3d::Quaternion<T> &a,
    const cslibs_math_3d::Quaternion<T> &b) {
  cslibs_math_3d::Quaternion<T> q;
  q.x() = a.x() - b.x();
  q.y() = a.y() - b.y();
  q.z() = a.z() - b.z();
  q.w() = a.w() - b.w();
  return q;
}

template <typename T>
inline cslibs_math_3d::Quaternion<T> operator*(
    const cslibs_math_3d::Quaternion<T> &a,
    const cslibs_math_3d::Quaternion<T> &b) {
  cslibs_math_3d::Quaternion<T> q;
  q.x() = a.w() * b.x() + a.x() * b.w() + a.y() * b.z() - a.z() * b.y();
  q.y() = a.w() * b.y() + a.y() * b.w() + a.z() * b.x() - a.x() * b.z();
  q.z() = a.w() * b.z() + a.z() * b.w() + a.x() * b.y() - a.y() * b.x();
  q.w() = a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z();
  return q;
}

template <typename T>
inline cslibs_math_3d::Vector3<T> operator*(
    const cslibs_math_3d::Quaternion<T> &q,
    const cslibs_math_3d::Vector3<T> &v) {
  cslibs_math_3d::Quaternion<T> inverse = q.invert();
  cslibs_math_3d::Quaternion<T> qv(v(0), v(1), v(2), T());
  qv = q * qv * inverse;
  return cslibs_math_3d::Vector3<T>(qv.x(), qv.y(), qv.z());
}

#endif  // CSLIBS_MATH_3D_QUATERNION_HPP
