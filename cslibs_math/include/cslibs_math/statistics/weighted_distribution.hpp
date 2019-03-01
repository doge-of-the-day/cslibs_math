#ifndef CSLIBS_MATH_WEIGHTED_DISTRIBUTION_HPP
#define CSLIBS_MATH_WEIGHTED_DISTRIBUTION_HPP

#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>

#include <cslibs_math/statistics/limit_eigen_values.hpp>
#include <cslibs_math/common/sqrt.hpp>

namespace cslibs_math {
namespace statistics {
template<std::size_t Dim, typename T = double, std::size_t lambda_ratio_exponent = 0>
class EIGEN_ALIGN16 WeightedDistribution {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t         = Eigen::aligned_allocator<WeightedDistribution<Dim, T, lambda_ratio_exponent>>;

    using Ptr                 = std::shared_ptr<WeightedDistribution<Dim, T, lambda_ratio_exponent>> ;
    using sample_t            = Eigen::Matrix<T, Dim, 1>;
    using sample_transposed_t = Eigen::Matrix<T, 1, Dim>;
    using covariance_t        = Eigen::Matrix<T, Dim, Dim>;
    using eigen_values_t      = Eigen::Matrix<T, Dim, 1>;
    using eigen_vectors_t     = Eigen::Matrix<T, Dim, Dim>;
    using allocator_t         = Eigen::aligned_allocator<WeightedDistribution>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr T sqrt_2_M_PI = cslibs_math::common::sqrt(2.0 * M_PI);


    WeightedDistribution() :
        sample_count_(0),
        mean_(sample_t::Zero()),
        correlated_(covariance_t::Zero()),
        W_(0.0),
        W_sq_(0.0),
        covariance_(covariance_t::Zero()),
        information_matrix_(covariance_t::Zero()),
        eigen_values_(eigen_values_t::Zero()),
        eigen_vectors_(eigen_vectors_t::Zero()),
        determinant_(0.0),
        dirty_(false)
    {
    }

    inline WeightedDistribution(std::size_t  sample_count,
                                T            w,
                                T            w_sq,
                                sample_t     mean,
                                covariance_t correlated) :
        sample_count_(sample_count),
        mean_(mean),
        correlated_(correlated),
        W_(w),
        W_sq_(w_sq),
        covariance_(covariance_t::Zero()),
        information_matrix_(covariance_t::Zero()),
        eigen_values_(eigen_values_t::Zero()),
        eigen_vectors_(eigen_vectors_t::Zero()),
        determinant_(0.0),
        dirty_(true)
    {
    }

    WeightedDistribution(const WeightedDistribution &other)  :
        sample_count_(other.sample_count_),
        mean_(other.mean_),
        correlated_(other.correlated_),
        W_(other.W_),
        W_sq_(other.W_sq_),
        covariance_(other.covariance_),
        information_matrix_(other.information_matrix_),
        eigen_values_(other.eigen_values_),
        eigen_vectors_(other.eigen_vectors_),
        determinant_(other.determinant_),
        dirty_(other.dirty_)
    {
    }

    WeightedDistribution& operator=(const WeightedDistribution &other)
    {
        sample_count_           = other.sample_count_;
        mean_                   = other.mean_;
        correlated_             = other.correlated_;
        W_                      = other.W_;
        W_sq_                   = other.W_sq_;

        covariance_             = other.covariance_;
        information_matrix_     = other.information_matrix_;
        eigen_values_           = other.eigen_values_;
        eigen_vectors_          = other.eigen_vectors_;
        determinant_            = other.determinant_;

        dirty_                  = other.dirty_;
        return *this;
    }

    WeightedDistribution(WeightedDistribution &&other) :
        sample_count_(other.sample_count_),
        mean_(std::move(other.mean_)),
        correlated_(std::move(other.correlated_)),
        W_(other.W_),
        W_sq_(other.W_sq_),
        covariance_(std::move(other.covariance_)),
        information_matrix_(std::move(other.information_matrix_)),
        eigen_values_(std::move(other.eigen_values_)),
        eigen_vectors_(std::move(other.eigen_vectors_)),
        determinant_(other.determinant_),
        dirty_(other.dirty_)
    {
    }

    WeightedDistribution& operator=(WeightedDistribution &&other)
    {
        sample_count_           = other.sample_count_;
        mean_                   = std::move(other.mean_);
        correlated_             = std::move(other.correlated_);
        W_                      = other.W_;
        W_sq_                   = other.W_sq_;

        covariance_             = std::move(other.covariance_);
        information_matrix_     = std::move(other.information_matrix_);
        eigen_values_           = std::move(other.eigen_values_);
        eigen_vectors_          = std::move(other.eigen_vectors_);
        determinant_            = other.determinant_;

        dirty_                  = other.dirty_;
        return *this;
    }

    inline void reset()
    {
        mean_       = sample_t::Zero();
        covariance_ = covariance_t::Zero();
        correlated_ = covariance_t::Zero();
        eigen_vectors_ = eigen_vectors_t::Zero();
        eigen_values_  = eigen_values_t::Zero();
        W_ = 0.0;
        W_sq_ = 0.0;
        sample_count_ = 0;
        dirty_ = true;
    }

    /// Modification
    inline void add(const sample_t &p, const T w)
    {
        if(w <= 0.0)
            return;

        const T _W = W_ + w;
        mean_ = (mean_ * W_ + p * w) / _W;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                correlated_(i, j) = (correlated_(i, j) * W_ + w * p(i) * p(j)) / static_cast<T>(_W);
            }
        }
        ++sample_count_;
        W_     = _W;
        W_sq_ +=  w*w;
        dirty_ =  true;
    }


    inline WeightedDistribution& operator+=(const WeightedDistribution &other)
    {
        const T _W = W_ + other.W_;
        mean_ = (mean_ * W_ + other.mean_ * other.W_) / _W;
        correlated_ = (correlated_ * W_ +  other.correlated_ * other.W_) / _W;
        W_ = _W;
        W_sq_ += other.W_sq_;
        sample_count_ += other.sample_count_;
        dirty_ = true;
        return *this;
    }

    /// Distribution properties
    inline bool valid() const
    {
        return sample_count_ > Dim;
    }

    inline std::size_t getSampleCount() const
    {
        return sample_count_;
    }

    inline T getWeight() const
    {
        return W_;
    }

    inline T getWeightSQ() const
    {
        return W_sq_;
    }

    inline sample_t getMean() const
    {
        return mean_;
    }

    inline void getMean(sample_t &mean) const
    {
        mean = mean_;
    }

    inline covariance_t getCorrelated() const
    {
        return correlated_;
    }

    inline covariance_t getCovariance() const
    {
        auto update_return_covariance = [this](){update(); return covariance_;};
        return valid() ? (dirty_ ? update_return_covariance() : covariance_) : covariance_t::Zero();
    }

    inline void getCovariance(covariance_t &covariance) const
    {
        auto update_return_covariance = [this](){update(); return covariance_;};
        covariance = valid() ? (dirty_ ? update_return_covariance() : covariance_) : covariance_t::Zero();
    }

    inline covariance_t getInformationMatrix() const
    {
        auto update_return_information = [this](){update(); return information_matrix_;};
        return valid() ? (dirty_ ? update_return_information() : information_matrix_) : covariance_t::Zero();
    }

    inline void getInformationMatrix(covariance_t &information_matrix) const
    {
        auto update_return_information = [this](){update(); return information_matrix_;};
        information_matrix = valid() ? (dirty_ ? update_return_information() : information_matrix_) : covariance_t::Zero();
    }

    inline eigen_values_t getEigenValues(const bool abs = false) const
    {
        auto update_return_eigen = [this, abs]() {
            if(dirty_) update();
            return abs ? eigen_values_.cwiseAbs() : eigen_values_;
        };
        return valid() ?  update_return_eigen() : eigen_values_t::Zero();
    }

    inline void getEigenValues(eigen_values_t &eigen_values,
                               const bool abs = false) const
    {
        auto update_return_eigen = [this, abs]() {
            if(dirty_) update();
            return abs ? eigen_values_.cwiseAbs() : eigen_values_;
        };
        eigen_values = valid() ?  update_return_eigen() : eigen_values_t::Zero();
    }

    inline eigen_vectors_t getEigenVectors() const
    {
        auto update_return_eigen = [this]() {
            if(dirty_) update();
            return eigen_vectors_;
        };
        return valid() ? update_return_eigen() : eigen_vectors_t::Zero();
    }

    inline void getEigenVectors(eigen_vectors_t &eigen_vectors) const
    {
        auto update_return_eigen = [this]() {
            if(dirty_) update();
            return eigen_vectors_;
        };
        eigen_vectors = valid() ? update_return_eigen() : eigen_vectors_t::Zero();
    }

    /// Evaluation
    inline T denominator() const
    {
        auto update_return = [this](){
            if(dirty_) update();
            return 1.0 / (determinant_ * sqrt_2_M_PI);
        };
        return valid() ? update_return() : 0.0;
    }

    inline T sample(const sample_t &p) const
    {
        auto update_sample = [this, &p]() {
            if(dirty_) update();
            const sample_t  q        = p - mean_;
            const T exponent    = -0.5 * static_cast<T>(static_cast<sample_transposed_t>(q.transpose()) * information_matrix_ * q);
            const T denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        };
        return valid() ? update_sample() : 0.0;
    }

    inline T sample(const sample_t &p,
                    sample_t &q) const
    {
        auto update_sample = [this, &p, &q]() {
            if(dirty_) update();
            q = p - mean_;
            const T exponent    = -0.5 * static_cast<T>(static_cast<sample_transposed_t>(q.transpose()) * information_matrix_ * q);
            const T denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        };
        return valid() ? update_sample() : 0.0;
    }

    inline T sampleNonNormalized(const sample_t &p) const
    {
        auto update_sample = [this, &p]() {
            if(dirty_) update();
            const sample_t  q   = p - mean_;
            const T exponent    = -0.5 * static_cast<T>(static_cast<sample_transposed_t>(q.transpose()) * information_matrix_ * q);
            return std::exp(exponent);
        };
        return valid() ? update_sample() : 0.0;
    }

    inline T sampleNonNormalized(const sample_t &p,
                                      sample_t &q) const
    {
        auto update_sample = [this, &p, &q]() {
            if(dirty_) update();
            q = p - mean_;
            const T exponent    = -0.5 * static_cast<T>(static_cast<sample_transposed_t>(q.transpose()) * information_matrix_ * q);
            return std::exp(exponent);
        };
        return valid() ? update_sample() : 0.0;
    }

    inline void merge(const WeightedDistribution&)
    {
    }

private:
    std::size_t               sample_count_;
    sample_t                  mean_;
    covariance_t              correlated_;
    T                         W_;
    T                         W_sq_;

    mutable covariance_t      covariance_;
    mutable covariance_t      information_matrix_;
    mutable eigen_values_t    eigen_values_;
    mutable eigen_vectors_t   eigen_vectors_;
    mutable T                 determinant_;

    mutable bool              dirty_;

    inline void update() const
    {
        const T scale = W_ / (W_ - W_sq_ / W_);
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                covariance_(i, j) = (correlated_(i, j) - (mean_(i) * mean_(j))) * scale;
                covariance_(j, i) =  covariance_(i, j);
            }
        }

        LimitEigenValues<Dim, T, lambda_ratio_exponent>::apply(covariance_);

        Eigen::EigenSolver<covariance_t> solver;
        solver.compute(covariance_);
        eigen_vectors_ = solver.eigenvectors().real();
        eigen_values_  = solver.eigenvalues().real();

        information_matrix_ = covariance_.inverse();
        determinant_        = covariance_.determinant();
        dirty_              = false;
    }

}__attribute__ ((aligned (16)));

template<typename T, std::size_t lambda_ratio_exponent>
class WeightedDistribution<1, T, lambda_ratio_exponent>
{
public:
    static constexpr T sqrt_2_M_PI = cslibs_math::common::sqrt(2.0 * M_PI);

    inline WeightedDistribution() :
        sample_count_(0),
        mean_(0.0),
        variance_(0.0),
        standard_deviation_(0.0),
        squared_(0.0),
        dirty_(false),
        W_(0.0),
        W_sq_(0.0)
    {
    }

    inline WeightedDistribution(std::size_t sample_count,
                                T           w,
                                T           w_sq,
                                T           mean,
                                T           squared) :
        sample_count_(sample_count),
        mean_(mean),
        squared_(squared),
        variance_(0.0),
        standard_deviation_(0.0),
        W_(w),
        W_sq_(w_sq),
        dirty_(true)
    {
    }

    inline WeightedDistribution(const WeightedDistribution &other) = default;
    inline WeightedDistribution(WeightedDistribution &&other) = default;
    inline WeightedDistribution& operator=(const WeightedDistribution &other) = default;

    inline void reset()
    {
        mean_               = 0.0;
        squared_            = 0.0;
        variance_           = 0.0;
        standard_deviation_ = 0.0;
        dirty_              = false;
        W_                  = 0.0;
        W_sq_               = 0.0;
        sample_count_       = 0;
    }

    inline void add(const T s, const T w)
    {
        const T _W = W_ + w;
        mean_    = (mean_ * W_ + s * w) / _W;
        squared_ = (squared_ * W_ + s*s * w) / _W;
        W_       = _W;
        W_sq_   += w*w;
        ++sample_count_;
        dirty_ = true;
    }

    inline WeightedDistribution & operator += (const WeightedDistribution &other)
    {
        const T _W = W_ + other.W_;
        mean_    = (mean_ * W_ + other.mean_ * other.W_) / _W;
        squared_ = (squared_ * W_ +  other.squared_ * other.W_) / _W;
        W_       = _W;
        W_sq_   += other.W_sq_;
        sample_count_ += other.sample_count_;
        dirty_ = true;
        return *this;
    }

    inline std::size_t getSampleCount() const
    {
        return sample_count_;
    }

    inline T getWeight() const
    {
        return W_;
    }

    inline T getWeightSQ() const
    {
        return W_sq_;
    }

    inline T getMean() const
    {
        return mean_;
    }

    inline T getVariance() const
    {
        if (dirty_) update();
        return variance_;
    }

    inline T getStandardDeviation() const
    {
        if (dirty_) update();
        return standard_deviation_;
    }

    inline T sample(const T s) const
    {
        if (dirty_) update();
        const T d = 2 * variance_;
        const T x = (s - mean_);
        return std::exp(-0.5 * x * x / d) / (sqrt_2_M_PI * standard_deviation_);
    }

    inline T sampleNonNormalized(const T s) const
    {
        if (dirty_) update();
        const T d = 2 * variance_;
        const T x = (s - mean_);
        return std::exp(-0.5 * x * x / d);
    }

    inline void merge(const WeightedDistribution&)
    {
    }

private:
    std::size_t     sample_count_;
    T               mean_;
    mutable T       variance_;
    mutable T       standard_deviation_;
    T               squared_;
    bool            dirty_;
    T               W_;
    T               W_sq_;

    inline void update() const
    {
        const T scale = W_ / (W_ - W_sq_ / W_);
        variance_ = (squared_ - mean_ * mean_) * scale;
        standard_deviation_ = std::sqrt(variance_);
        dirty_ = false;
    }

}__attribute__ ((aligned (16)));
}
}

template<std::size_t D, typename T, std::size_t L>
std::ostream & operator << (std::ostream &out, const cslibs_math::statistics::WeightedDistribution<D,T,L> &d)
{
    out << d.getMean() << "\n";
    out << d.getCovariance() << "\n";
    out << d.getWeight() << "\n";
    return out;
}

template<typename T, std::size_t L>
std::ostream & operator << (std::ostream &out, const cslibs_math::statistics::WeightedDistribution<1,T,L> &d)
{
    out << d.getMean() << "\n";
    out << d.getVariance() << "\n";
    out << d.getWeight() << "\n";
    return out;
}

#endif // CSLIBS_MATH_WEIGHTED_DISTRIBUTION_HPP
