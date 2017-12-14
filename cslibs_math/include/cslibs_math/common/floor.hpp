#ifndef CSLIBS_MATH_FLOOR_HPP
#define CSLIBS_MATH_FLOOR_HPP

#include <cmath>

namespace cslibs_math {
namespace common {
template<typename T>
T floor(const T a)
{
    return a >= static_cast<T>(0.0) ? std::floor(a) : (std::floor(a) - static_cast<T>(1.0));
}
}
}

#endif // CSLIBS_MATH_FLOOR_HPP
