#ifndef CSLIBS_MATH_2D_TF_HPP
#define CSLIBS_MATH_2D_TF_HPP

#include <tf/tf.h>

#include <cslibs_math_3d/linear/covariance.hpp>
#include <cslibs_math_3d/linear/transform.hpp>

namespace cslibs_math_3d {
namespace conversion {
inline Vector3d from(const tf::Vector3 &v)
{
    return Vector3d(v.x(), v.y(), v.z());
}

inline tf::Vector3 from(const Vector3d &v)
{
    return tf::Vector3(v(0), v(1), v(2));
}

inline Quaternion from(const tf::Quaternion &q)
{
    return Quaternion(q.x(), q.y(), q.z(), q.w());
}

inline tf::Quaternion from(const Quaternion &q)
{
    return tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}

inline Transform3d from(const tf::Transform &t)
{
    return Transform3d(from(t.getOrigin()),
                       from(t.getRotation()));
}

inline tf::Transform from(const Transform3d &t)
{
    return tf::Transform(from(t.rotation()),
                         from(t.translation()));
}

inline void from(const std::vector<tf::Transform> &src,
                 std::vector<Transform3d>         &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const tf::Transform &t){return from(t);});
}

inline void from(const std::vector<tf::Vector3> &src,
                 std::vector<Transform3d> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const tf::Vector3 &v){return from(v);});
}

inline void from(const std::vector<Transform3d> &src,
                 std::vector<tf::Transform> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const Transform3d &t){return from(t);});
}

inline void from(const std::vector<Vector3d> &src,
                 std::vector<tf::Vector3> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const Vector3d &v){return from(v);});
}
}
}

#endif // CSLIBS_MATH_2D_TF_HPP
