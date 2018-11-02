#ifndef CSLIBS_MATH_SQRT_HPP
#define CSLIBS_MATH_SQRT_HPP

#include <limits>

namespace cslibs_math {
namespace common {
template <typename T>
constexpr T sqrtNewtonRaphson(T x, T curr, T prev)
{
    return curr == prev ? curr : sqrtNewtonRaphson(x, T(0.5) * (curr + x / curr), curr);
}

template <typename T>
constexpr T sqrt(T x)
{
    return x >= T(0.0) && x < std::numeric_limits<T>::infinity() ?
        sqrtNewtonRaphson(x, x, T(0.0)) : std::numeric_limits<T>::quiet_NaN();
}
}
}

#endif // CSLIBS_MATH_SQRT_HPP
