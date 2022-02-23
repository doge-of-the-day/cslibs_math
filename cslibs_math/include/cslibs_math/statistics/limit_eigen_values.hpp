#ifndef CSLIBS_MATH_LIMIT_COVARIANCE_HPP
#define CSLIBS_MATH_LIMIT_COVARIANCE_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

namespace cslibs_math {
namespace statistics {

template <typename T, std::size_t exp>
struct LambdaRatio {
  constexpr const static T value = 0.1 * LambdaRatio<T, exp - 1ul>::value;
};
template <typename T>
struct LambdaRatio<T, 0ul> {
  constexpr const static T value = 1.0;
};

template <typename T, std::size_t Dim, std::size_t lambda_ratio_exponent>
/**
 * @brief The LimitEigenValues struct is used to assure, that eigen values are
 * not too small. This will probihit the distribution to collapse in one
 * dimension, since eigen values represent the length of the eigen vectors,
 * which build a local orthonormal system.
 */
struct LimitEigenValues {
  static_assert(Dim < static_cast<std::size_t>(std::numeric_limits<int>::max()), "'Dim' cannot exceed integer maximum.");

  const static constexpr T lambda_ratio =
      LambdaRatio<T, lambda_ratio_exponent>::value;

  using matrix_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;
  using eigen_values_t = Eigen::Matrix<T, static_cast<int>(Dim), 1>;
  using eigen_vectors_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;

  inline static void apply(matrix_t &matrix_io) {
    Eigen::EigenSolver<matrix_t> solver;
    solver.compute(matrix_io);
    const auto eigen_values = solver.eigenvalues().real();
    const auto eigen_vectors = solver.eigenvectors().real();

    const T lambda = lambda_ratio * eigen_values.maxCoeff();
    matrix_t Lambda = matrix_t::Zero();
    for (std::size_t i = 0; i < Dim; ++i) {
      Lambda(i, i) = std::abs(eigen_values(i)) < std::abs(lambda)
                         ? lambda
                         : eigen_values(i);
    }
    matrix_io = eigen_vectors * Lambda * (eigen_vectors.transpose()).format();
  }
};

template <typename T, std::size_t Dim>
struct LimitEigenValues<T, Dim, 0ul> {
  static_assert(Dim < static_cast<std::size_t>(std::numeric_limits<int>::max()), "'Dim' cannot exceed integer maximum.");

  using matrix_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;
  using eigen_values_t = Eigen::Matrix<T, static_cast<int>(Dim), 1>;
  using eigen_vectors_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;

  inline static void apply(matrix_t &) {}
};

template <typename T, std::size_t Dim>
struct LimitEigenValuesByZero {
  static_assert(Dim < static_cast<std::size_t>(std::numeric_limits<int>::max()), "'Dim' cannot exceed integer maximum.");

  using matrix_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;
  using eigen_values_t = Eigen::Matrix<T, static_cast<int>(Dim), 1>;
  using eigen_vectors_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;

  inline static void apply(matrix_t &matrix_io) {
    Eigen::EigenSolver<matrix_t> solver;
    solver.compute(matrix_io);
    const auto eigen_values = solver.eigenvalues().real();
    const auto eigen_vectors = solver.eigenvectors().real();

    matrix_t Lambda = matrix_t::Zero();
    for (std::size_t i = 0; i < Dim; ++i) {
      Lambda(i, i) = std::max(eigen_values(i), T());  // T() = zero
    }
    matrix_io = eigen_vectors * Lambda * eigen_vectors.transpose();
  }
};
}  // namespace statistics
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_LIMIT_COVARIANCE_HPP
