#ifndef CSLIBS_MATH_WEIGHTED_DISTRIBUTION_HPP
#define CSLIBS_MATH_WEIGHTED_DISTRIBUTION_HPP

#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>

#include <cslibs_math/statistics/limit_covariance.hpp>

namespace cslibs_math {
namespace statistics {
template<std::size_t Dim, std::size_t lamda_ratio_exponent = 0>
class WeightedDistribution {
public:
    using Ptr = std::shared_ptr<WeightedDistribution<Dim, lamda_ratio_exponent>> ;

    using sample_t        = Eigen::Matrix<double, Dim, 1>;
    using covariance_t    = Eigen::Matrix<double, Dim, Dim>;
    using eigen_values_t  = Eigen::Matrix<double, Dim, 1>;
    using eigen_vectors_t = Eigen::Matrix<double, Dim, Dim>;

    static constexpr double sqrt_2_M_PI = std::sqrt(2 * M_PI);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    WeightedDistribution() :
        sample_count_(0),
        mean_(sample_t::Zero()),
        correlated_(covariance_t::Zero()),
        W_(0.0),
        W_1_(0.0),
        covariance_(covariance_t::Zero()),
        information_matrix_(covariance_t::Zero()),
        eigen_values_(eigen_values_t::Zero()),
        eigen_vectors_(eigen_vectors_t::Zero()),
        determinant_(0.0),
        dirty_(false)
    {
    }

    WeightedDistribution(const WeightedDistribution &other)             = default;
    WeightedDistribution& operator=(const WeightedDistribution &other)  = default;

    inline void reset()
    {
        mean_       = sample_t::Zero();
        covariance_ = covariance_t::Zero();
        correlated_ = covariance_t::Zero();
        eigen_vectors_ = eigen_vectors_t::Zero();
        eigen_values_  = eigen_values_t::Zero();
        W_ = 1;
        W_1_ = 0;
        sample_count_ = 0;
        dirty_ = true;
    }

    /// Modification
    inline void add(const sample_t &p, const double w)
    {
        W_  += w;
        mean_ = (mean_ * W_1_ + w * p) / W_;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                correlated_(i, j) = (correlated_(i, j) * W_1_ + w * p(i) * p(j)) / static_cast<double>(W_);
            }
        }
        ++sample_count_;
        W_1_ = W_;
        dirty_ = true;
    }


    inline WeightedDistribution& operator+=(const WeightedDistribution &other)
    {
        mean_ = (mean_ * W_ + other.mean_ * other.W_) / (W_ + other.W_);
        correlated_ = (correlated_ * W_ +  other.correlated_ * other.W_) / (W_ + other.W_);
        W_ += other.W_;
        W_1_ = W_;
        sample_count_ += other.sample_count_;
        dirty_ = true;
        return *this;
    }

    /// Distribution properties
    inline std::size_t getSampleCount() const
    {
        return sample_count_;
    }

    inline double getWeight() const
    {
        return W_;
    }

    inline sample_t getMean() const
    {
        return mean_;
    }

    inline void getMean(sample_t &mean) const
    {
        mean = mean_;
    }

    inline covariance_t getCovariance() const
    {
        auto update_return_covariance = [this](){update(); return covariance_;};
        return sample_count_ >= 3 ? (dirty_ ? update_return_covariance() : covariance_) : covariance_t::Zero();
    }

    inline void getCovariance(covariance_t &covariance) const
    {
        auto update_return_covariance = [this](){update(); return covariance_;};
        covariance =  sample_count_ >= 3 ? (dirty_ ? update_return_covariance() : covariance_) : covariance_t::Zero();
    }

    inline covariance_t getInformationMatrix() const
    {
        auto update_return_information = [this](){update(); return information_matrix_;};
        return sample_count_ >= 3 ? (dirty_ ? update_return_information() : information_matrix_) : covariance_t::Zero();
    }

    inline void getInformationMatrix(covariance_t &information_matrix) const
    {
        auto update_return_information = [this](){update(); return information_matrix_;};
        information_matrix = sample_count_ >= 3 ? (dirty_ ? update_return_information() : information_matrix_) : covariance_t::Zero();
    }

    inline eigen_values_t getEigenValues(const bool abs = false) const
    {
        auto update_return_eigen = [this, abs]() {
            if(dirty_) update();
            return abs ? eigen_values_.cwiseAbs() : eigen_values_;
        };
        return sample_count_ >= 3 ?  update_return_eigen() : eigen_values_t::Zero();
    }

    inline void getEigenValues(eigen_values_t &eigen_values,
                               const bool abs = false) const
    {
        auto update_return_eigen = [this, abs]() {
            if(dirty_) update();
            return abs ? eigen_values_.cwiseAbs() : eigen_values_;
        };
        eigen_values = sample_count_ >= 3 ?  update_return_eigen() : eigen_values_t::Zero();
    }

    inline eigen_vectors_t getEigenVectors() const
    {
        auto update_return_eigen = [this]() {
            if(dirty_) update();
            return eigen_vectors_;
        };
        return sample_count_ >= 3 ? update_return_eigen() : eigen_vectors_t::Zero();
    }

    inline void getEigenVectors(eigen_vectors_t &eigen_vectors) const
    {
        auto update_return_eigen = [this]() {
            if(dirty_) update();
            return eigen_vectors_;
        };
        eigen_vectors = sample_count_ >= 3 ? update_return_eigen() : eigen_vectors_t::Zero();
    }

    /// Evaluation
    inline double denominator() const
    {
        auto update_return = [this](){
            if(dirty_) update();
            return 1.0 / (determinant_ * sqrt_2_M_PI);
        };
        return sample_count_ >= 3 ? update_return() : 0.0;
    }

    inline double sample(const sample_t &p) const
    {
        auto update_sample = [this, &p]() {
            if(dirty_) update();
            const sample_t  q        = p - mean_;
            const double exponent    = -0.5 * double(q.transpose() * information_matrix_ * q);
            const double denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        };
        return sample_count_ >= 3 ? update_sample() : 0.0;
    }

    inline double sample(const sample_t &p,
                         sample_t &q) const
    {
        auto update_sample = [this, &p, &q]() {
            if(dirty_) update();
            q = p - mean_;
            const double exponent    = -0.5 * double(q.transpose() * information_matrix_ * q);
            const double denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        };
        return sample_count_ >= 3 ? update_sample() : 0.0;
    }

    inline double sampleNonNormalized(const sample_t &p) const
    {
        auto update_sample = [this, &p]() {
            if(dirty_) update();
            const sample_t  q        = p - mean_;
            const double exponent    = -0.5 * double(q.transpose() * information_matrix_ * q);
            return std::exp(exponent);
        };
        return sample_count_ >= 3 ? update_sample() : 0.0;
    }

    inline double sampleNonNormalized(const sample_t &p,
                                      sample_t &q) const
    {
        auto update_sample = [this, &p, &q]() {
            if(dirty_) update();
            q = p - mean_;
            const double exponent    = -0.5 * double(q.transpose() * information_matrix_ * q);
            return std::exp(exponent);
        };
        return sample_count_ >= 3 ? update_sample() : 0.0;
    }

private:
    std::size_t               sample_count_;
    sample_t                  mean_;
    covariance_t              correlated_;
    double                    W_;
    double                    W_1_;            /// actual amount of points in distribution

    mutable covariance_t      covariance_;
    mutable covariance_t      information_matrix_;
    mutable eigen_values_t    eigen_values_;
    mutable eigen_vectors_t   eigen_vectors_;
    mutable double            determinant_;

    mutable bool              dirty_;

    inline void update() const
    {
        const double scale = 1.0 / W_;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                covariance_(i, j) = (correlated_(i, j) - (mean_(i) * mean_(j))) * scale;
                covariance_(j, i) =  covariance_(i, j);
            }
        }

        LimitCovariance<Dim, lamda_ratio_exponent>::apply(covariance_);

        Eigen::EigenSolver<covariance_t> solver;
        solver.compute(covariance_);
        eigen_vectors_ = solver.eigenvectors().real();
        eigen_values_  = solver.eigenvalues().real();

        information_matrix_ = covariance_.inverse();
        determinant_        = covariance_.determinant();
        dirty_              = false;
    }

};

template<std::size_t lamda_ratio_exponent>
class WeightedDistribution<1, lamda_ratio_exponent>
{
public:
   static constexpr double sqrt_2_M_PI = std::sqrt(2 * M_PI);

    inline WeightedDistribution() :
        sample_count_(0),
        mean_(0.0),
        variance_(0.0),
        standard_deviation_(0.0),
        squared_(0.0),
        dirty_(false),
        W_(1.0),
        W_1_(0.0)
    {
    }

    inline WeightedDistribution(const WeightedDistribution &other) = default;
    inline WeightedDistribution(WeightedDistribution &&other) = default;
    inline WeightedDistribution& operator=(const WeightedDistribution &other) = default;

    inline void reset()
    {
        mean_           = 0.0;
        squared_        = 0.0;
        variance_       = 0.0;
        standard_deviation_ = 0.0;
        dirty_          = false;
        W_              = 1.0;
        W_1_            = 0.0;
        sample_count_   = 0;
    }

    inline void add(const double s, const double w)
    {
        W_      += w;
        mean_    = (mean_ * W_1_ + s * w) / W_;
        squared_ = (squared_ * W_1_ + s*s * w) / W_;
        ++sample_count_;
        W_1_ = W_;
        dirty_ = true;
    }

    inline WeightedDistribution & operator += (const WeightedDistribution &other)
    {
        mean_    = (mean_ * W_ + other.mean_ * other.W_) / (W_ + other.W_);
        squared_ = (squared_ * W_ +  other.squared_ * other.W_) / (W_ + other.W_);
        W_ += other.W_;
        W_1_ = W_;
        sample_count_ += other.sample_count_;
        dirty_ = true;
        return *this;
    }

    inline std::size_t getSampleCount() const
    {
        return sample_count_;
    }

    inline double getMean() const
    {
        return mean_;
    }

    inline double getVariance() const
    {
        return dirty_ ? updateReturnVariance() : variance_;
    }

    inline double getStandardDeviation() const
    {
        return dirty_ ? updateReturnStandardDeviation() : standard_deviation_;
    }

    inline double sample(const double s) const
    {
        const double d = 2 * (dirty_ ? updateReturnVariance() : variance_);
        const double x = (s - mean_);
        return std::exp(-0.5 * x * x / d) / (sqrt_2_M_PI * standard_deviation_);
    }

    inline double sampleNonNormalized(const double s) const
    {
        const double d = 2 * (dirty_ ? updateReturnVariance() : variance_);
        const double x = (s - mean_);
        return std::exp(-0.5 * x * x / d);
    }

private:
    std::size_t     sample_count_;
    double          mean_;
    mutable double  variance_;
    mutable double  standard_deviation_;
    double          squared_;
    bool            dirty_;
    double          W_;
    double          W_1_;

    inline double updateReturnVariance() const
    {
        variance_ = squared_ - mean_ * mean_;
        standard_deviation_ = std::sqrt(variance_);
        return variance_;
    }

    inline double updateReturnStandardDeviation() const
    {
        variance_ = squared_ - mean_ * mean_;
        standard_deviation_ = std::sqrt(variance_);
        return standard_deviation_;
    }
};

}
}

#endif // CSLIBS_MATH_WEIGHTED_DISTRIBUTION_HPP
