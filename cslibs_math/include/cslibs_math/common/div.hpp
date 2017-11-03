#ifndef CSLIBS_MATH_DIV_HPP
#define CSLIBS_MATH_DIV_HPP

namespace cslibs_math {
namespace common {
template<typename T>
T div(const T a, const T b)
{
    assert(b > 0);
    return a < 0 ? (a / b - 1) : (a / b);
}
}
}

#endif // CSLIBS_MATH_DIV_HPP
