#ifndef CSLIBS_MATH_DISTRIBUTION_HPP
#define CSLIBS_MATH_DISTRIBUTION_HPP

#undef NDEBUG
#include <assert.h>
#include <memory>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>

namespace cslibs_math {
namespace statistics {
template<std::size_t Dim, bool limit_covariance = false>
class Distribution {
public:
    using Ptr = std::shared_ptr<Distribution<Dim, limit_covariance>>;

    using sample_t        = Eigen::Matrix<double, Dim, 1>;
    using covariance_t    = Eigen::Matrix<double, Dim, Dim>;
    using eigen_values_t  = Eigen::Matrix<double, Dim, 1>;
    using eigen_vectors_t = Eigen::Matrix<double, Dim, Dim>;

    static constexpr double sqrt_2_M_PI = std::sqrt(2 * M_PI);
    static constexpr double lambda_ratio = 1e-2;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline Distribution() :
        mean_(sample_t::Zero()),
        correlated_(covariance_t::Zero()),
        n_(1),
        n_1_(0),
        covariance_(covariance_t::Zero()),
        inverse_covariance_(covariance_t::Zero()),
        eigen_values_(eigen_values_t::Zero()),
        eigen_vectors_(eigen_vectors_t::Zero()),
        determinant_(0.0),
        dirty_(false),
        dirty_eigen_(false)
    {
    }

    inline Distribution(const Distribution &other) = default;
    inline Distribution(Distribution &&other) = default;
    inline Distribution& operator=(const Distribution &other) = default;

    inline void reset()
    {
        mean_ = sample_t::Zero();
        covariance_ = covariance_t::Zero();
        correlated_ = covariance_t::Zero();
        n_ = 1;
        n_1_ = 0;
        dirty_ = true;
        dirty_eigen_ = true;
    }

    /// Modification
    inline void add(const sample_t &p)
    {
        mean_ = (mean_ * n_1_ + p) / n_;
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                correlated_(i, j) = (correlated_(i, j) * n_1_ + p(i) * p(j)) / static_cast<double>(n_);
            }
        }
        ++n_;
        ++n_1_;
        dirty_ = true;
        dirty_eigen_ = true;
    }

    inline Distribution& operator+=(const sample_t &p)
    {
        add(p);
        return *this;
    }

    inline Distribution& operator+=(const Distribution &other)
    {
        std::size_t _n = n_1_ + other.n_1_;
        sample_t   _mean = (mean_ * n_1_ + other.mean_ * other.n_1_) / static_cast<double>(_n);
        covariance_t  _corr = (correlated_ * n_1_ + other.correlated_ * other.n_1_) / static_cast<double>(_n);
        n_   = _n + 1;
        n_1_ = _n;
        mean_ = _mean;
        correlated_ = _corr;
        dirty_ = true;
        dirty_eigen_ = true;
        return *this;
    }

    /// Distribution properties
    inline std::size_t getN() const
    {
        return n_1_;
    }

    inline sample_t getMean() const
    {
        return mean_;
    }

    inline void getMean(sample_t &_mean) const
    {
        _mean = mean_;
    }

    inline covariance_t getCovariance() const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            return covariance_;
        }
        return covariance_t::Zero();
    }

    inline void getCovariance(covariance_t &covariance) const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            covariance = covariance_;
        } else {
            covariance = covariance_t::Zero();
        }
    }

    inline covariance_t getInformationMatrix() const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            return inverse_covariance_;
        }
        return covariance_t::Zero();
    }

    inline void getInformationMatrix(covariance_t &inverse_covariance) const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            inverse_covariance = inverse_covariance_;
        } else {
            inverse_covariance = covariance_t::Zero();
        }
    }

    inline eigen_values_t getEigenValues(const bool abs = false) const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            if(dirty_eigen_)
                updateEigen();

            if(abs)
                return eigen_values_.cwiseAbs();
            else
                return eigen_values_;
        }
        return eigen_values_t::Zero();
    }

    inline void getEigenValues(eigen_values_t &eigen_values,
                               const bool abs = false) const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            if(dirty_eigen_)
                updateEigen();

            if(abs)
                eigen_values = eigen_values_.cwiseAbs();
            else
                eigen_values = eigen_values_;
        } else {
            eigen_values = eigen_values_t::Zero();
        }
    }

    inline eigen_vectors_t getEigenVectors() const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            if(dirty_eigen_)
                updateEigen();

            return eigen_vectors_;
        }
        return eigen_vectors_t::Zero();
    }

    inline void getEigenVectors(eigen_vectors_t &eigen_vectors) const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            if(dirty_eigen_)
                updateEigen();

            eigen_vectors = eigen_vectors_;
        } else {
            eigen_vectors = eigen_vectors_t::Zero();
        }
    }

    /// Evaluation
    inline double sample(const sample_t &p) const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            sample_t  q = p - mean_;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance_ * q);
            double denominator = 1.0 / (covariance_.determinant() * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        }
        return 0.0;
    }

    inline double sample(const sample_t &p,
                         sample_t &q) const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            q = p - mean_;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance_ * q);
            double denominator = 1.0 / (determinant_ * sqrt_2_M_PI);
            return denominator * std::exp(exponent);
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const sample_t &p) const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();

            sample_t  q = p - mean_;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance_ * q);
            return std::exp(exponent);
        }
        return 0.0;
    }

    inline double sampleNonNormalized(const sample_t &p,
                                      sample_t &q) const
    {
        if(n_1_ >= 2) {
            if(dirty_)
                update();
            q = p - mean_;
            double exponent = -0.5 * double(q.transpose() * inverse_covariance_ * q);
            return std::exp(exponent);
        }
        return 0.0;
    }

private:
    sample_t                    mean_;
    covariance_t                correlated_;
    std::size_t                 n_;
    std::size_t                 n_1_;            /// actual amount of points in distribution

    mutable covariance_t         covariance_;
    mutable covariance_t         inverse_covariance_;
    mutable eigen_values_t       eigen_values_;
    mutable eigen_vectors_t      eigen_vectors_;
    mutable double               determinant_;

    mutable bool                 dirty_;
    mutable bool                 dirty_eigen_;

    inline void update() const
    {
        double scale = n_1_ / static_cast<double>(n_1_ - 1);
        for(std::size_t i = 0 ; i < Dim ; ++i) {
            for(std::size_t j = i ; j < Dim ; ++j) {
                covariance_(i, j) = (correlated_(i, j) - (mean_(i) * mean_(j))) * scale;
                covariance_(j, i) = covariance_(i, j);
            }
        }

        if(limit_covariance) {
            if(dirty_eigen_)
                updateEigen();

            double max_lambda = std::numeric_limits<double>::lowest();
            for(std::size_t i = 0 ; i < Dim ; ++i) {
                if(eigen_values_(i) > max_lambda)
                    max_lambda = eigen_values_(i);
            }
            covariance_t Lambda = covariance_t::Zero();
            double l = max_lambda * lambda_ratio;
            for(std::size_t i = 0 ; i < Dim; ++i) {
                if(fabs(eigen_values_(i)) < fabs(l)) {
                    Lambda(i,i) = l;
                } else {
                    Lambda(i,i) = eigen_values_(i);
                }
            }
            covariance_ = eigen_vectors_ * Lambda * eigen_vectors_.transpose();
            inverse_covariance_ = eigen_vectors_ * Lambda.inverse() * eigen_vectors_.transpose();
        } else {
            inverse_covariance_ = covariance_.inverse();
        }

        determinant_ = covariance_.determinant();
        dirty_ = false;
        dirty_eigen_ = true;
    }

    inline void updateEigen() const
    {
        Eigen::EigenSolver<covariance_t> solver;
        solver.compute(covariance_);
        eigen_vectors_ = solver.eigenvectors().real();
        eigen_values_  = solver.eigenvalues().real();
        dirty_eigen_ = false;
    }
};

template<>
class Distribution<1, false>
{
public:
   static constexpr double sqrt_2_M_PI = std::sqrt(2 * M_PI);

    inline Distribution() :
        mean_(0.0),
        variance_(0.0),
        standard_deviation_(0.0),
        squared_(0.0),
        dirty_(false),
        n_(1),
        n_1_(0)
    {
    }

    inline Distribution(const Distribution &other) = default;
    inline Distribution(Distribution &&other) = default;
    inline Distribution& operator=(const Distribution &other) = default;

    inline void reset()
    {
        mean_       = 0.0;
        squared_    = 0.0;
        variance_   = 0.0;
        standard_deviation_ = 0.0;
        dirty_      = false;
        n_          = 1;
        n_1_        = 0;
    }

    inline void add(const double s)
    {
        mean_    = (mean_ * n_1_ + s) / n_;
        squared_ = (squared_ * n_1_ + s*s) / n_;
        ++n_;
        ++n_1_;
        dirty_ = true;
    }

    inline Distribution & operator += (const double s)
    {
        mean_    = (mean_ * n_1_ + s) / n_;
        squared_ = (squared_ * n_1_ + s*s) / n_;
        ++n_;
        ++n_1_;
        dirty_ = true;
        return *this;
    }

    inline Distribution & operator += (const Distribution &other)
    {
        std::size_t _n = n_1_ + other.n_1_;
        double  _mean = (mean_ * n_1_ + other.mean_ * other.n_1_) / static_cast<double>(_n);
        double  _squared = (_squared * n_1_ + other.squared_ * other.n_1_) / static_cast<double>(_n);
        n_   = _n + 1;
        n_1_ = _n;
        mean_ = _mean;
        dirty_ = true;
        return *this;
    }

    inline std::size_t getN() const
    {
        return n_1_;
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
    double          mean_;
    mutable double  variance_;
    mutable double  standard_deviation_;
    double          squared_;
    bool            dirty_;
    std::size_t     n_;
    std::size_t     n_1_;

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

template<>
class Distribution<1, true>
{
public:
   static constexpr double sqrt_2_M_PI = std::sqrt(2 * M_PI);

    inline Distribution() :
        mean_(0.0),
        variance_(0.0),
        standard_deviation_(0.0),
        squared_(0.0),
        dirty_(false),
        n_(1),
        n_1_(0)
    {
    }

    inline Distribution(const Distribution &other) = default;
    inline Distribution(Distribution &&other) = default;
    inline Distribution& operator=(const Distribution &other) = default;

    inline void reset()
    {
        mean_       = 0.0;
        squared_    = 0.0;
        variance_   = 0.0;
        standard_deviation_ = 0.0;
        dirty_      = false;
        n_          = 1;
        n_1_        = 0;
    }

    inline void add(const double s)
    {
        mean_    = (mean_ * n_1_ + s) / n_;
        squared_ = (squared_ * n_1_ + s*s) / n_;
        ++n_;
        ++n_1_;
        dirty_ = true;
    }

    inline Distribution & operator += (const double s)
    {
        mean_    = (mean_ * n_1_ + s) / n_;
        squared_ = (squared_ * n_1_ + s*s) / n_;
        ++n_;
        ++n_1_;
        dirty_ = true;
        return *this;
    }

    inline Distribution & operator += (const Distribution &other)
    {
        std::size_t _n = n_1_ + other.n_1_;
        double  _mean = (mean_ * n_1_ + other.mean_ * other.n_1_) / static_cast<double>(_n);
        double  _squared = (_squared * n_1_ + other.squared_ * other.n_1_) / static_cast<double>(_n);
        n_   = _n + 1;
        n_1_ = _n;
        mean_ = _mean;
        dirty_ = true;
        return *this;
    }

    inline std::size_t getN() const
    {
        return n_1_;
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
    double          mean_;
    mutable double  variance_;
    mutable double  standard_deviation_;
    double          squared_;
    bool            dirty_;
    std::size_t     n_;
    std::size_t     n_1_;

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

#endif /* CSLIBS_MATH_DISTRIBUTION_HPP */
