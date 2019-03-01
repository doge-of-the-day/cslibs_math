#ifndef CSLIBS_MATH_2D_COVARIANCE_2D_HPP
#define CSLIBS_MATH_2D_COVARIANCE_2D_HPP

#include <cslibs_math/linear/matrix.hpp>

namespace cslibs_math_2d {
template <typename T>
using Covariance3d = cslibs_math::linear::Matrix<T, 3, 3>;
}

#endif // CSLIBS_MATH_2D_COVARIANCE_2D_HPP
