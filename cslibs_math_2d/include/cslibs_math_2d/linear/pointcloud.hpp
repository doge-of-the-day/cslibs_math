#ifndef CSLIBS_MATH_2D_POINTCLOUD2D_HPP
#define CSLIBS_MATH_2D_POINTCLOUD2D_HPP

#include <cslibs_math_2d/linear/point.hpp>
#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math/linear/pointcloud.hpp>

namespace cslibs_math_2d {
template <typename T = double>
using Pointcloud2d = cslibs_math::linear::Pointcloud<Point2d<T>>;
}

#endif // CSLIBS_MATH_2D_POINTCLOUD2D_HPP
