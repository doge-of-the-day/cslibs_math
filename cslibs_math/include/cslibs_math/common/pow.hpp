#ifndef CSLIBS_MATH_POW_HPP
#define CSLIBS_MATH_POW_HPP
namespace cslibs_math {
namespace common {
namespace detail {
template <unsigned int E, typename T>
struct pow;

template <typename T>
struct pow<0ul, T> {
  inline static constexpr T eval(const T value) { return 1.0; }
};

template <typename T>
struct pow<1ul, T> {
  inline static constexpr T eval(const T value) { return value; }
};

template <unsigned int E, typename T = double>
struct pow {
  inline static constexpr  T eval(const T value) {
    return value * (pow<E - 1ul, T>::eval(value));
  }
};
}

template <unsigned int E, typename T = double>
inline constexpr T pow(const T value) {
  return detail::pow<E, T>::eval(value);
}

template <unsigned int E, typename T = double>
inline constexpr T pow2() {
  return detail::pow<E, T>::eval(T{2});
}
}  // namespace common
}  // namespace cslibs_math
#endif  // CSLIBS_MATH_POW_HPP