#ifndef CSLIBS_MATH_EXP_HPP
#define CSLIBS_MATH_EXP_HPP

#include <cslibs_math/common/pow.hpp>

namespace cslibs_math {
namespace approx {
template<unsigned int Accuracy_T, typename T = double>
inline constexpr T exp(const T value)
{
    const auto x = T{1} + value / common::pow2<Accuracy_T, T>();
    return common::pow<Accuracy_T + 1ul>(x);
}
}  // namespace common
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_EXP_HPP
