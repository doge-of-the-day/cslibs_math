#ifndef CSLIBS_MATH_3D_POINTCLOUD_3D_HPP
#define CSLIBS_MATH_3D_POINTCLOUD_3D_HPP

#include <cslibs_math/linear/pointcloud.hpp>
#include <cslibs_math_3d/linear/transform.hpp>
#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
template <typename T>
using Pointcloud3d = cslibs_math::linear::Pointcloud<Point3d<T>>;
template <typename T>
using PointcloudRGB3d = cslibs_math::linear::Pointcloud<PointRGB3d<T>>;
}

#endif // CSLIBS_MATH_3D_POINTCLOUD_3D_HPP
