#ifndef CSLIBS_MATH_WEIGHTED_ANGULAR_MEAN_HPP
#define CSLIBS_MATH_WEIGHTED_ANGULAR_MEAN_HPP

#include <complex>
#include <cslibs_math/common/angle.hpp>
#include <eigen3/Eigen/Core>
#include <memory>

namespace cslibs_math {
namespace statistics {
template <typename T>
class EIGEN_ALIGN16 WeightedAngularMean {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using allocator_t = Eigen::aligned_allocator<WeightedAngularMean<T>>;

  using Ptr = std::shared_ptr<WeightedAngularMean<T>>;
  using complex_t = Eigen::Matrix<T, 2, 1>;

  inline WeightedAngularMean() = default;
  inline WeightedAngularMean(const WeightedAngularMean &other) = default;
  inline WeightedAngularMean(WeightedAngularMean &&other) = default;

  inline WeightedAngularMean &operator=(const WeightedAngularMean &other) = default;
  inline WeightedAngularMean &operator=(WeightedAngularMean &&other) = default;

  void reset() {
    dirty_ = true;
    mean_ = T();
    complex_mean_(0) = T();
    complex_mean_(1) = T();
    W_ = T();
  }

  inline void add(const T rad, const T w) {
    if (w == T()) return;

    T _W = W_ + w;
    complex_mean_ =
        (complex_mean_ * W_ + complex_t(std::cos(rad), std::sin(rad)) * w) / _W;
    W_ = _W;
    dirty_ = true;
  }

  inline WeightedAngularMean &operator+=(const WeightedAngularMean &other) {
    T _W = W_ + other.W_;
    complex_mean_ = (complex_mean_ * W_ + other.complex_mean_ * other.W_) / _W;
    W_ = _W;
    dirty_ = true;
    return *this;
  }

  inline T getWeight() const { return W_; }

  inline T getMean() const {
    if (dirty_) {
      mean_ = std::atan2(complex_mean_(1), complex_mean_(0));
      dirty_ = false;
    }
    return mean_;
  }

  inline void getMean(T &mean) {
    if (dirty_) {
      mean_ = std::atan2(complex_mean_(1), complex_mean_(0));
      dirty_ = false;
    }
    mean = mean_;
  }

  inline T getCovariance() const {
    return -2.0 * std::log(std::hypot(complex_mean_(0), complex_mean_(1)));
  }

 private:
  mutable bool dirty_{false};
  mutable T mean_{0};
  complex_t complex_mean_{0, 0};
  T W_{0};
};
}  // namespace statistics
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_WEIGHTED_ANGULAR_MEAN_HPP
