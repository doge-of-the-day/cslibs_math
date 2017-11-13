#ifndef CSLIBS_MATH_3D_POINTCLOUD_3D_HPP
#define CSLIBS_MATH_3D_POINTCLOUD_3D_HPP

#include <cslibs_math_3d/linear/box.hpp>
#include <cslibs_math_3d/linear/transform.hpp>

#include <cslibs_math/linear/pointcloud.hpp>

#include <memory>
#include <vector>

namespace cslibs_math_3d {
using Pointcloud3d = cslibs_math::linear::Pointcloud<Point3d>;
inline Box3d boundingBox(const Pointcloud3d &points)
{
    return Box3d(points.min(), points.max());
}

inline void transform(const Transform3d &t, Pointcloud3d &points)
{
    std::for_each(points.begin(), points.end(),
                  [&t](Point2d &p){p = t * p;});
}
}



#endif // CSLIBS_MATH_2D_POINTCLOUD_3D_HPP
