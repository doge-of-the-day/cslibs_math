#ifndef CSLIBS_MATH_ROS_CONVERSION_3D_HPP
#define CSLIBS_MATH_ROS_CONVERSION_3D_HPP

#include <tf/tf.h>

#include <cslibs_math_3d/linear/covariance.hpp>
#include <cslibs_math_3d/linear/transform.hpp>

namespace cslibs_math_ros {
namespace tf {
namespace conversion_3d {
inline cslibs_math_3d::Vector3d from(const ::tf::Vector3 &v)
{
    return cslibs_math_3d::Vector3d(v.x(), v.y(), v.z());
}

inline ::tf::Vector3 from(const cslibs_math_3d::Vector3d &v)
{
    return ::tf::Vector3(v(0), v(1), v(2));
}

inline ::tf::Quaternion from(const cslibs_math_3d::Quaternion &q)
{
    return ::tf::Quaternion(q.x(), q.y(), q.z(), q.w());
}

inline cslibs_math_3d::Quaternion from(const ::tf::Quaternion &q)
{
    return cslibs_math_3d::Quaternion(q.x(), q.y(), q.z(), q.w());
}

inline cslibs_math_3d::Transform3d from(const ::tf::Transform &t)
{
    return cslibs_math_3d::Transform3d(from(t.getOrigin()),
                                       from(t.getRotation()));
}

inline ::tf::Transform from(const cslibs_math_3d::Transform3d &t)
{
    return ::tf::Transform(from(t.rotation()),
                           from(t.translation()));
}

inline void from(const std::vector<::tf::Transform> &src,
                 std::vector<cslibs_math_3d::Transform3d>         &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const ::tf::Transform &t){return from(t);});
}

inline void from(const std::vector<::tf::Vector3> &src,
                 std::vector<cslibs_math_3d::Transform3d> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const ::tf::Vector3 &v){return from(v);});
}

inline void from(const std::vector<cslibs_math_3d::Transform3d> &src,
                 std::vector<::tf::Transform> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_3d::Transform3d &t){return from(t);});
}

inline void from(const std::vector<cslibs_math_3d::Vector3d> &src,
                 std::vector<::tf::Vector3> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_3d::Vector3d &v){return from(v);});
}
}
}
}
#endif // CSLIBS_MATH_ROS_CONVERSION_3D_HPP
