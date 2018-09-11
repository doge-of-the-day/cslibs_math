#ifndef CSLIBS_MATH_WEIGHTED_ANGULAR_MEAN_HPP
#define CSLIBS_MATH_WEIGHTED_ANGULAR_MEAN_HPP

#include <memory>
#include <complex>
#include <cslibs_math/common/angle.hpp>
#include <eigen3/Eigen/Core>

namespace cslibs_math {
namespace statistics {
class EIGEN_ALIGN16 WeightedAngularMean {
public:
  using Ptr         = std::shared_ptr<WeightedAngularMean>;
  using complex_t   = Eigen::Vector2d;
  using allocator_t = Eigen::aligned_allocator<WeightedAngularMean>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  WeightedAngularMean() :
    dirty_(false),
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
    dirty_ = other.dirty_;
    mean_  = other.mean_;
    complex_mean_ = other.complex_mean_;
    W_ = other.W_;
    return *this;
  }

  WeightedAngularMean& operator=(WeightedAngularMean &&other)
  {
    dirty_ = other.dirty_;
    mean_  = other.mean_;
    complex_mean_ = other.complex_mean_;
    W_ = other.W_;
    return *this;
  }

  void reset()
  {
    dirty_           = true;
    mean_            = 0.0;
    complex_mean_(0) = 0.0;
    complex_mean_(1) = 0.0;
    W_               = 0.0;
  }

  inline void add(const double rad, const double w)
  {
    if(w == 0.0)
      return;

    double _W = W_ + w;
    complex_mean_ = (complex_mean_ * W_ + complex_t(std::cos(rad), std::sin(rad)) * w) / _W;
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
      mean_ = std::atan2(complex_mean_(1), complex_mean_(0));
      dirty_ = false;
    }
    return mean_;
  }

  inline void getMean(double &mean) {
    if(dirty_) {
      mean_ = std::atan2(complex_mean_(1), complex_mean_(0));
      dirty_ = false;
    }
    mean = mean_;
  }

  inline double getCovariance() const
  {
    return -2.0 * std::log(std::hypot(complex_mean_(0), complex_mean_(1)));
  }

private:
  mutable bool    dirty_;
  mutable double  mean_;
  complex_t complex_mean_;
  double  W_;
};
}
}
#endif // CSLIBS_MATH_WEIGHTED_ANGULAR_MEAN_HPP
