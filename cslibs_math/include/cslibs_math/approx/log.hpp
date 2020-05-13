#ifndef CSLIBS_MATH_LOG_HPP
#define CSLIBS_MATH_LOG_HPP

#include <cmath>

namespace cslibs_math {
namespace approx {
inline double log2(const double d) {
     int exponent;
     double fraction = std::frexp(d, &exponent);
     return (exponent-1) + 2.3 * (fraction - 0.5);
}
inline float log2(const float f) {
     int exponent;
     float fraction = std::frexp(f, &exponent);
     return (exponent-1) + 2.3 * (fraction - 0.5);
}
}  // namespace common
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_LOG_HPP