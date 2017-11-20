#ifndef CSLIBS_MATH_DISTRIBUTION_1D_HPP
#define CSLIBS_MATH_DISTRIBUTION_1D_HPP

#include <cstddef>
#include <cmath>

namespace cslibs_math {
namespace statistics {
class Distribution
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

#endif // CSLIBS_MATH_DISTRIBUTION_1D_HPP
