#ifndef CSLIBS_MATH_SQRT_HPP
#define CSLIBS_MATH_SQRT_HPP

#include <limits>
#include <cslibs_math/utility/traits.hpp>

namespace cslibs_math {
namespace common {
template <typename T>
constexpr T sqrtNewtonRaphson(T x, T curr, T prev)
{
    return curr == prev ? curr : sqrtNewtonRaphson(x, utility::traits<T>::Half * (curr + x / curr), curr);
}

template <typename T>
constexpr T sqrt(T x)
{
    return x >= T() && x < std::numeric_limits<T>::infinity() ?
        sqrtNewtonRaphson(x, x, T()) : std::numeric_limits<T>::quiet_NaN();
}
}
}

#endif // CSLIBS_MATH_SQRT_HPP
