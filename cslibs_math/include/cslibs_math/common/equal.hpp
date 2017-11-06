#ifndef CSLIBS_MATH_EQUAL_HPP
#define CSLIBS_MATH_EQUAL_HPP

#include <limits>
#include <cmath>

namespace cslibs_math {
namespace common {
template<typename T>
inline bool eq(const T a, const T b)
{
    return std::abs(a - b) < std::numeric_limits<T>::epsilon();
}

template<typename T>
inline bool ge(const T a, const T b)
{
    return eq(a,b) || a > b;
}

template<typename T>
inline bool le(const T a, const T b)
{
    return eq(a,b) || a < b;
}

template<typename T>
inline bool neq(const T a, const T b)
{
    return !eq(a,b);
}
}
}

#endif // EQUAL_HPP
