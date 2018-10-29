#ifndef CSLIBS_MATH_2D_VECTOR_2D_HPP
#define CSLIBS_MATH_2D_VECTOR_2D_HPP

#include <cslibs_math/linear/vector.hpp>
#include <ostream>

namespace cslibs_math_2d {
using Vector2d = cslibs_math::linear::Vector<double,2>;

inline double angle(const Vector2d &v)
{
    return std::atan2(v(1), v(0));
}
}

namespace std {
inline bool isnormal(const cslibs_math_2d::Vector2d &v)
{
    return std::isnormal(v(0)) &&
           std::isnormal(v(1));
}
}

inline std::ostream & operator << (std::ostream &out, const cslibs_math_2d::Vector2d &v)
{
    out << "[" << v(0) << "," << v(1) << "]";
    return out;
}

#endif // CSLIBS_MATH_2D_VECTOR_2D_HPP
