#ifndef CSLIBS_MATH_LOG_HPP
#define CSLIBS_MATH_LOG_HPP

#include <cmath>

namespace cslibs_math {
namespace approx {
double log2_fast(double d) {
     int exponent;
     double fraction = std::frexp(d, &exponent);
     return (exponent-1) + 2.3 * (fraction - 0.5);
}
}  // namespace common
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_LOG_HPP