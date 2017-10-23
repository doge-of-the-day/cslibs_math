#ifndef CSLIBS_MATH_2D_TF_HPP
#define CSLIBS_MATH_2D_TF_HPP

#include <tf/tf.h>

#include <cslibs_math_2d/types/covariance.hpp>
#include <cslibs_math_2d/types/transform.hpp>

namespace cslibs_math_2d {
namespace conversion {
inline Vector2d from(const tf::Vector3 &v)
{
    return Vector2d(v.x(), v.y());
}

inline Transform2d from(const tf::Transform &t)
{
    return Transform2d(from(t.getOrigin()),
                       tf::getYaw(t.getRotation()));
}

inline tf::Vector3 from(const Vector2d &v)
{
    return tf::Vector3(v.x(), v.y(), 0.0);
}

inline tf::Transform from(const Transform2d &t)
{
    return tf::Transform(tf::createQuaternionFromYaw(t.yaw()),
                         from(t.translation()));
}

inline void from(const std::vector<tf::Transform> &src,
                 std::vector<Transform2d> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const tf::Transform &t){return from(t);});
}

inline void from(const std::vector<tf::Vector3> &src,
                 std::vector<Transform2d> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const tf::Vector3 &v){return from(v);});
}

inline void from(const std::vector<Transform2d> &src,
                 std::vector<tf::Transform> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const Transform2d &t){return from(t);});
}

inline void from(const std::vector<Vector2d> &src,
                 std::vector<tf::Vector3> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [](const Vector2d &v){return from(v);});
}
}
}

#endif // CSLIBS_MATH_2D_TF_HPP
