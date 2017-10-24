#ifndef CSLIBS_MATH_MOD_HPP
#define CSLIBS_MATH_MOD_HPP

namespace cslibs_math {
namespace common {
template<typename T>
T mod(const T a, const T b)
{
  return a < 0 ? (a+b)%b : a%b;
}
}
}

#endif // CSLIBS_MATH_MOD_HPP
