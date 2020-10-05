#ifndef CSLIBS_MATH_FRACTIONAL_HPP
#define CSLIBS_MATH_FRACTIONAL_HPP

#include <cmath>
#include <limits>
#include <type_traits>

namespace cslibs_math {
namespace approx {

template <typename T>
class Fractional {
 public:
  static_assert(std::is_floating_point<T>::value,
                "Type must be floating point.");

  inline Fractional() = default;
  explicit Fractional(const T value)
      : fraction_{std::frexp(value, &exponent_)} {}
  explicit Fractional(const T fraction, const int exponent)
      : exponent_{exponent}, fraction_{fraction} {}

  inline T value() const { return std::ldexp(fraction_, exponent_); }

  inline T fraction() const { return fraction_; }

  inline int exponent() const { return exponent_; }

 private:
  int exponent_{0};
  T fraction_{0};
};
}  // namespace approx
}  // namespace cslibs_math

template <typename T>
inline cslibs_math::approx::Fractional<T> operator == (
    const cslibs_math::approx::Fractional<T> &a,
    const cslibs_math::approx::Fractional<T> &b) {
  return a.fraction() == b.fraction() && a.exponent() == b.exponent();
}

template <typename T>
inline cslibs_math::approx::Fractional<T> operator < (
    const cslibs_math::approx::Fractional<T> &a,
    const cslibs_math::approx::Fractional<T> &b) {
  return a.fraction() < b.fraction() && a.exponent() <= b.exponent();
}

template <typename T>
inline cslibs_math::approx::Fractional<T> operator*(
    const cslibs_math::approx::Fractional<T> &a,
    const cslibs_math::approx::Fractional<T> &b) {
  auto factor = cslibs_math::approx::Fractional<T>(a.fraction() * b.fraction());
  auto exponent = a.exponent() + b.exponent() + factor.exponent();
  return cslibs_math::approx::Fractional<T>(factor.fraction(), exponent);
}

template <typename T>
inline cslibs_math::approx::Fractional<T> operator/(
    const cslibs_math::approx::Fractional<T> &a,
    const cslibs_math::approx::Fractional<T> &b) {
  auto factor = cslibs_math::approx::Fractional<T>(a.fraction() / b.fraction());
  auto exponent = a.exponent() - b.exponent() + factor.exponent();
  return cslibs_math::approx::Fractional<T>(factor.fraction(), exponent);
}

template <typename T>
inline cslibs_math::approx::Fractional<T> operator+(
    const cslibs_math::approx::Fractional<T> &a,
    const cslibs_math::approx::Fractional<T> &b) {
  const auto exponent_diff = a.exponent() - b.exponent();
  const auto scale = std::ldexp(1.0, exponent_diff);
  return cslibs_math::approx::Fractional<T>(b.fraction() + scale * a.fraction(),
                                            b.exponent());
}

template <typename T>
inline cslibs_math::approx::Fractional<T> operator-(
    const cslibs_math::approx::Fractional<T> &a,
    const cslibs_math::approx::Fractional<T> &b) {
  const auto exponent_diff = a.exponent() - b.exponent();
  const auto scale = std::ldexp(1.0, exponent_diff);
  return cslibs_math::approx::Fractional<T>(scale * a.fraction() - b.fraction(),
                                            b.exponent());
}

#endif  // CSLIBS_MATH_FRACTIONAL_HPP