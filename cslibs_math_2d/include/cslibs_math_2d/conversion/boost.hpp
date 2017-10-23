#ifndef CSLIBS_MATH_2D_BOOST_HPP
#define CSLIBS_MATH_2D_BOOST_HPP

#include <cslibs_math_2d/types/line.hpp>
#include <cslibs_math_2d/types/point.hpp>

#include <cslibs_boost_geometry/types.hpp>

namespace cslibs_math_2d {
namespace conversion {

inline Point2d from(const cslibs_boost_geometry::types::Point2d &p)
{
    return Point2d(p.x(), p.y());
}

inline Line2d from(const cslibs_boost_geometry::types::Line2d &l)
{
    return {from(l.first), from(l.second)};
}


inline cslibs_boost_geometry::types::Point2d from(const Point2d &p)
{
    return cslibs_boost_geometry::types::Point2d(p.x(), p.y());
}

inline cslibs_boost_geometry::types::Line2d from(const Line2d &l)
{
    return {from(l.first), from(l.second)};
}

inline void from(const cslibs_boost_geometry::types::PointSet2d &src,
                 std::vector<Point2d> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [t](const cslibs_boost_geometry::types::Point2d &p){return from(p);});
}

inline void from(const cslibs_boost_geometry::types::Line2dSet &src,
                 std::vector<Line2d> &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [t](const cslibs_boost_geometry::types::Line2d &l){return from(l);});
}

inline void from(const std::vector<Point2d> &src,
                 cslibs_boost_geometry::types::PointSet2d &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [t](const Point2d &p){return from(p);});
}

inline void from(std::vector<Line2d> &src,
                 const cslibs_boost_geometry::types::Line2dSet &dst)
{
    dst.resize(src.size());
    std::transform(src.begin(), src.end(),
                   dst.begin(),
                  [t](const Line2d &l){return from(l);});
}

}
}



#endif // CSLIBS_MATH_2D_BOOST_HPP
