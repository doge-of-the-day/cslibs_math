#ifndef CSLIBS_MATH_EXP_HPP
#define CSLIBS_MATH_EXP_HPP

#include <cslibs_math/common/pow.hpp>

namespace cslibs_math {
namespace approx {
namespace detail {
template <unsigned int I, typename T = double>
struct square {
  inline static constexpr T eval(const T value) {
    return square<I - 1ul, T>::eval(value * value);
  }
};
template <typename T>
struct square<0ul, T> {
  inline static constexpr T eval(const T value) { return 1.0; }
};
template <typename T>
struct square<1ul, T> {
  inline static constexpr T eval(const T value) { return value * value; }
};
}  // namespace detail

template <unsigned int Accuracy_T, typename T>
inline constexpr T exp(const T value) {
  return detail::square<Accuracy_T, T>::eval(
      T{1.0} + value / common::pow2<Accuracy_T, T>());
}
}  // namespace approx
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_EXP_HPP
