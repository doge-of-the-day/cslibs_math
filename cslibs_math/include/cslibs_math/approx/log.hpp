#ifndef CSLIBS_MATH_LOG_HPP
#define CSLIBS_MATH_LOG_HPP

#include <cmath>
#include <limits>
#include <type_traits>

namespace cslibs_math {
namespace approx {
namespace detail {
template <typename T>
struct Base2 {
  inline static T log(const T value) { return std::log2(value); }
  inline static T exp(const T log) { return std::exp2(log); }
  inline static T log1p(const T value) { return std::log2(1.0 + value); }
  inline static T log1m(const T value) { return std::log2(1.0 - value); }
};
template <typename T>
struct Base10 {
  // this does not work with float properly, assuming that accuracy is too low
  // for tiny numbers
  inline static T log(const T value) { return std::log10(value); }
  inline static T exp(const T log) { return exp10(log); }
  inline static T log1p(const T value) { return std::log10(1.0 + value); }
  inline static T log1m(const T value) { return std::log10(1.0 - value); }
};
template <typename T>
struct BaseE {
  inline static T log(const T value) { return std::log(value); }
  inline static T exp(const T log) { return std::exp(log); }
  inline static T log1p(const T value) { return std::log1p(value); }
  inline static T log1m(const T value) { return std::log(1.0 - value); }
};
}  // namespace detail

template <typename T, typename Base_T = detail::Base2<T>>
class Log {
 public:
  static_assert(std::is_floating_point<T>::value,
                "Type must be floating point.");

  inline explicit Log(const T value) : value_{Base_T::log(value)} {}

  inline Log operator*(const T value) const {
    Log l;
    l.value_ = value_ + Base_T::log(value);
    return l;
  }

  inline Log operator/(const T value) const {
    Log l;
    l.value_ = value_ - Base_T::log(value);
    return l;
  }

  inline Log operator+(const T value) const {
    Log l;
    l.value_ = value_ + Base_T::log1p(value / Base_T::exp(value_));
    return l;
  }

  inline Log operator-(const T value) const {
    Log l;
    l.value_ = value_ + Base_T::log1m(value / Base_T::exp(value_));
    return l;
  }

  inline Log operator*(const Log log) const {
    Log l;
    l.value_ = value_ + log.value_;
    return l;
  }

  inline Log operator/(const Log log) const {
    Log l;
    l.value_ = value_ - log.value_;
    return l;
  }

  inline Log operator+(const Log log) const {
    Log l;
    l.value_ = value_ + Base_T::log1p(Base_T::exp(log.value_ - value_));
    return l;
  }

  inline Log operator-(const Log log) const {
    Log l;
    l.value_ = value_ + Base_T::log1m(Base_T::exp(log.value_ - value_));
    return l;
  }

  inline T value() const { return value(); }

  inline T exp() const { return Base_T::exp(value_); }

 private:
  Log() = default;

  T value_{-std::numeric_limits<T>::infinity()};
};

// https://en.wikipedia.org/wiki/List_of_logarithmic_identities
// https://en.wikipedia.org/wiki/Log_probability
/* log(x + y) = log(x + x * y/x)
 *            = log(x + x * exp(log(y/x)))
 *            = log(x * (1 + exp(log(y) - log(x))))
 *            = log(x) + log(1 + exp(log(y) - log(x)))
 *            = x' + log(1 + exp(y' - x'))
 */

}  // namespace approx
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_LOG_HPP