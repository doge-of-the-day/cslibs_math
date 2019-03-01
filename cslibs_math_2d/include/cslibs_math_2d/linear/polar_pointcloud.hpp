#ifndef CSLIBS_MATH_2D_POLAR_POINTCLOUD_HPP
#define CSLIBS_MATH_2D_POLAR_POINTCLOUD_HPP

#include <cslibs_math/linear/pointcloud.hpp>
#include <cslibs_math_2d/linear/polar_point.hpp>

namespace cslibs_math_2d {
template <typename T>
using PolarPointlcoud2d = cslibs_math::linear::Pointcloud<PolarPoint2d<T>>;
}
#endif // CSLIBS_MATH_2D_POLAR_POINTCLOUD_HPP
