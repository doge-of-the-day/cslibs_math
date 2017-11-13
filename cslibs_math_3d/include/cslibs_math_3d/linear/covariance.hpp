#ifndef CSLIBS_MATH_3D_COVARIANCE_3D_HPP
#define CSLIBS_MATH_3D_COVARIANCE_3D_HPP

#include <cslibs_math/linear/matrix.hpp>

namespace cslibs_math_3d {
using Covariance3d = cslibs_math::linear::Matrix<double, 6, 6>;
}

#endif // CSLIBS_MATH_3D_COVARIANCE_3D_HPP
