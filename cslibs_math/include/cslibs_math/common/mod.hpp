#ifndef CSLIBS_MATH_MOD_HPP
#define CSLIBS_MATH_MOD_HPP

namespace cslibs_math {
namespace common {
template<typename T>
T mod(const T a, const T b)
{
    auto r = [b](const T x) { return x < 0 ? x + b : x;};
    return r(a % b);
}
}
}

#endif // CSLIBS_MATH_MOD_HPP
