#ifndef CSLIBS_MATH_ANGULAR_MEAN_HPP
#define CSLIBS_MATH_ANGULAR_MEAN_HPP

#include <cslibs_math/common/angle.hpp>
#include <eigen3/Eigen/Core>
#include <memory>

namespace cslibs_math {
namespace statistics {
template <typename T>
class EIGEN_ALIGN16 AngularMean {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using allocator_t = Eigen::aligned_allocator<AngularMean<T>>;

  using Ptr = std::shared_ptr<AngularMean<T>>;
  using complex = std::complex<T>;

  AngularMean() = default;
  AngularMean(const AngularMean &other) = default;
  AngularMean(AngularMean &&other) = default;

  AngularMean &operator=(const AngularMean &other) = default;

  AngularMean &operator=(AngularMean &&other) = default;

  void reset() {
    dirty_ = true;
    mean_ = T();
    complex_mean_ = complex(T(), T());
    n_ = 1ul;
    n_1_ = 0ul;
  }

  inline void add(const T rad) {
    complex_mean_ =
        (complex_mean_ * static_cast<T>(n_1_) + common::angle::toComplex(rad)) /
        static_cast<T>(n_);
    ++n_;
    ++n_1_;
    dirty_ = true;
  }

  inline AngularMean &operator+=(const AngularMean &other) {
    dirty_ = true;
    std::size_t _n = n_1_ + other.n_1_;
    complex_mean_ = (complex_mean_ * static_cast<T>(n_1_) +
                     other.complex_mean_ * static_cast<T>(other.n_1_)) /
                    static_cast<T>(_n);
    n_ = _n + 1;
    n_1_ = _n;
    dirty_ = true;
    return *this;
  }

  inline T getN() const { return n_1_; }

  inline T getMean() const {
    if (dirty_) {
      mean_ = common::angle::fromComplex(complex_mean_);
      dirty_ = false;
    }
    return mean_;
  }

  inline void getMean(T &mean) {
    if (dirty_) {
      mean_ = common::angle::fromComplex(complex_mean_);
      dirty_ = false;
    }
    mean = mean_;
  }

  inline T getVariance() const {
    return -std::log(complex_mean_.real() * complex_mean_.real() +
                     complex_mean_.imag() * complex_mean_.imag());
  }

  inline T getStandardDeviation() const { return std::sqrt(getVariance()); }

 private:
  mutable bool dirty_{true};
  mutable T mean_{0};
  complex complex_mean_{0, 0};
  std::size_t n_{1ul};
  std::size_t n_1_{0ul};
};
}  // namespace statistics
}  // namespace cslibs_math

#endif  // ANGULAR_MEAN_HPP
