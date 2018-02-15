#ifndef CSLIBS_MATH_WEIGHTED_ANGULAR_MEAN_HPP
#define CSLIBS_MATH_WEIGHTED_ANGULAR_MEAN_HPP

#include <memory>
#include <complex>
#include <cslibs_math/common/angle.hpp>

namespace cslibs_math {
namespace statistics {
class WeightedAngularMean {
public:
    using Ptr = std::shared_ptr<WeightedAngularMean>;
    using complex = std::complex<double>;

    WeightedAngularMean() :
        dirty_(true),
        mean_(0.0),
        complex_mean_(0.0, 0.0),
        W_(0.0)
    {
    }

    WeightedAngularMean(const WeightedAngularMean &other) :
        dirty_(other.dirty_),
        mean_(other.mean_),
        complex_mean_(other.complex_mean_),
        W_(other.W_)
    {
    }

    WeightedAngularMean(WeightedAngularMean &&other) :
        dirty_(other.dirty_),
        mean_(other.mean_),
        complex_mean_(std::move(other.complex_mean_)),
        W_(other.W_)
    {
    }

    WeightedAngularMean& operator=(const WeightedAngularMean &other)
    {
        if(this != &other) {
            dirty_ = other.dirty_;
            mean_  = other.mean_;
            complex_mean_ = other.complex_mean_;
            W_ = other.W_;
        }
        return *this;
    }

    WeightedAngularMean& operator=(WeightedAngularMean &&other)
    {
        if(this != &other) {
            dirty_ = other.dirty_;
            mean_  = other.mean_;
            complex_mean_ = other.complex_mean_;
            W_ = other.W_;
        }
        return *this;
    }

    void reset()
    {
        dirty_ = true;
        mean_ = 0.0;
        complex_mean_ = 0.0;
        W_ = 0.0;
    }

    inline void add(const double rad, const double w)
    {
        double _W = W_ + w;
        complex_mean_ = (complex_mean_ * W_ + common::angle::toComplex(rad) * w) / _W;
        W_ = _W;
        dirty_ = true;
    }

    inline WeightedAngularMean& operator += (const WeightedAngularMean& other)
    {
        double _W = W_ + other.W_;
        complex_mean_ = (complex_mean_ * W_ + other.complex_mean_ * other.W_) / _W;
        W_ = _W;
        dirty_ = true;
        return *this;
    }

    inline double getWeight() const
    {
        return W_;
    }

    inline double getMean() const
    {
        if(dirty_) {
            mean_ = common::angle::fromComplex(complex_mean_);
            dirty_ = false;
        }
        return mean_;
    }

    inline void getMean(double &mean) {
        if(dirty_) {
            mean_ = common::angle::fromComplex(complex_mean_);
            dirty_ = false;
        }
        mean = mean_;
    }

    inline double getCovariance() const
    {
        return -2.0 * std::log(std::hypot(complex_mean_.real(), complex_mean_.imag()));
    }

private:
    mutable bool    dirty_;
    mutable double  mean_;
    complex complex_mean_;
    double  W_;
}__attribute__ ((aligned (64)));
}
}
#endif // CSLIBS_MATH_WEIGHTED_ANGULAR_MEAN_HPP
