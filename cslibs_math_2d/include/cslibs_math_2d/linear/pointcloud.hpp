#ifndef CSLIBS_MATH_2D_POINTCLOUD2D_HPP
#define CSLIBS_MATH_2D_POINTCLOUD2D_HPP

#include <cslibs_math_2d/linear/box.hpp>
#include <cslibs_math_2d/linear/transform.hpp>

#include <cslibs_math/linear/pointcloud.hpp>

#include <memory>
#include <vector>

namespace cslibs_math_2d {
using Pointcloud2d = cslibs_math::linear::Pointcloud<Point2d>;
inline Box2d boundingBox(const Pointcloud2d &points)
{
    return Box2d(points.min(), points.max());
}

inline void transform(const Transform2d &t, Pointcloud2d &points)
{
    std::for_each(points.begin(), points.end(),
                  [&t](Point2d &p){p = t * p;});
}
}



#endif // CSLIBS_MATH_2D_POINTCLOUD2D_HPP
