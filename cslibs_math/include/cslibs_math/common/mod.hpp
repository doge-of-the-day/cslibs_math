#ifndef CSLIBS_MATH_MOD_HPP
#define CSLIBS_MATH_MOD_HPP

#include <assert.h>

#include <type_traits>

namespace cslibs_math {
namespace common {
template <typename T>
T mod(const T a, const T b) {
  static_assert(std::is_integral<T>::value, "Integral required.");

  assert(b > 0);
  auto r = [b](const T x) { return x < T() ? (x + b) : x; };
  return r(a % b);
}
}  // namespace common
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_MOD_HPP
