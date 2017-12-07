#ifndef CSLIBS_MATH_ROS_CONVERSION_2D_HPP
#define CSLIBS_MATH_ROS_CONVERSION_2D_HPP

#include <tf/tf.h>

#include <cslibs_math_2d/linear/covariance.hpp>
#include <cslibs_math_2d/linear/transform.hpp>

namespace cslibs_math_ros {
namespace tf {
namespace conversion_2d {
inline cslibs_math_2d::Vector2d from(const ::tf::Vector3 &v)
{
    return cslibs_math_2d::Vector2d(v.x(), v.y());
}

inline cslibs_math_2d::Transform2d from(const ::tf::Transform &t)
{
    return cslibs_math_2d::Transform2d(from(t.getOrigin()),
                          ::tf::getYaw(t.getRotation()));
}

inline ::tf::Vector3 from(const cslibs_math_2d::Vector2d &v)
{
    return ::tf::Vector3(v(0), v(1), 0.0);
}

inline ::tf::Transform from(const cslibs_math_2d::Transform2d &t)
{
    return ::tf::Transform(::tf::createQuaternionFromYaw(t.yaw()),
                          from(t.translation()));
}

inline void from(const std::vector<::tf::Transform> &src,
                 std::vector<cslibs_math_2d::Transform2d> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const ::tf::Transform &t){return from(t);});
}

inline void from(const std::vector<::tf::Vector3> &src,
                 std::vector<cslibs_math_2d::Transform2d> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                   [](const ::tf::Vector3 &v){return from(v);});
}

inline void from(const std::vector<cslibs_math_2d::Transform2d> &src,
                 std::vector<::tf::Transform> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_2d::Transform2d &t){return from(t);});
}

inline void from(const std::vector<cslibs_math_2d::Vector2d> &src,
                 std::vector<::tf::Vector3> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const cslibs_math_2d::Vector2d &v){return from(v);});
}
}
}
}

#endif // CSLIBS_MATH_ROS_CONVERSION_2D_HPP
