#ifndef CSLIBS_MATH_3D_VECTOR_3D_HPP
#define CSLIBS_MATH_3D_VECTOR_3D_HPP

#include <cslibs_math/linear/vector.hpp>
#include <ostream>

namespace cslibs_math_3d {
using Vector3d = cslibs_math::linear::Vector<double,3>;
}

inline std::ostream & operator << (std::ostream &out, const cslibs_math_3d::Vector3d &v)
{
    out << "[" << v(0) << "," << v(1) << "," << v(2) << "]";
    return out;
}

#endif // CSLIBS_MATH_3D_VECTOR_3D_HPP
