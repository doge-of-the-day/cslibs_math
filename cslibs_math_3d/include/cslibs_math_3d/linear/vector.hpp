#ifndef CSLIBS_MATH_3D_VECTOR_3D_HPP
#define CSLIBS_MATH_3D_VECTOR_3D_HPP

#include <cslibs_math/linear/vector.hpp>
#include <ostream>

namespace cslibs_math_3d {
template <typename T>
using Vector3d = cslibs_math::linear::Vector<T,3>;
}

namespace std {
template <typename T>
inline bool isnormal(const cslibs_math_3d::Vector3d<T> &v)
{
    return std::isnormal(v(0)) &&
           std::isnormal(v(1)) &&
           std::isnormal(v(2));
}
}

template <typename T>
inline std::ostream & operator << (std::ostream &out, const cslibs_math_3d::Vector3d<T> &v)
{
    out << "[" << v(0) << "," << v(1) << "," << v(2) << "]";
    return out;
}

#endif // CSLIBS_MATH_3D_VECTOR_3D_HPP
