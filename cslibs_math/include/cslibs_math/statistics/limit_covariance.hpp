#ifndef CSLIBS_MATH_LIMIT_COVARIANCE_HPP
#define CSLIBS_MATH_LIMIT_COVARIANCE_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>

namespace cslibs_math {
namespace statistics {

template<std::size_t exp>
struct LambdaRatio
{
    constexpr const static double value = 0.1 * LambdaRatio<exp - 1ul>::value;
};

template<>
struct LambdaRatio<0ul>
{
    constexpr const static double value = 1.0;
};

template<std::size_t Dim, std::size_t lambda_ratio_exponent>
struct LimitCovariance
{
    const static constexpr double lambda_ratio = LambdaRatio<lambda_ratio_exponent>::value;

    using covariance_t    = Eigen::Matrix<double, Dim, Dim>;
    using eigen_values_t  = Eigen::Matrix<double, Dim, 1>;
    using eigen_vectors_t = Eigen::Matrix<double, Dim, Dim>;

    static inline void apply(covariance_t    &covariance_io)
    {
        Eigen::EigenSolver<covariance_t> solver;
        solver.compute(covariance_io);
        const auto eigen_values  = solver.eigenvalues().real();
        const auto eigen_vectors = solver.eigenvectors().real();

        const double lambda = lambda_ratio * eigen_values.maxCoeff();
        covariance_t Lambda = covariance_t::Zero();
        for(std::size_t i = 0 ; i < Dim; ++i) {
            Lambda(i,i) = std::abs(eigen_values(i)) < std::abs(lambda) ? lambda : eigen_values(i);
        }
        covariance_io = eigen_vectors * Lambda * eigen_vectors.transpose();
    }
};

template<std::size_t Dim>
struct LimitCovariance<Dim, 0ul>
{
    using covariance_t    = Eigen::Matrix<double, Dim, Dim>;
    using eigen_values_t  = Eigen::Matrix<double, Dim, 1>;
    using eigen_vectors_t = Eigen::Matrix<double, Dim, Dim>;

    static inline void apply(covariance_t &)
    {

    }
};
}
}


#endif // CSLIBS_MATH_LIMIT_COVARIANCE_HPP
