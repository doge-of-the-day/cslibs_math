#ifndef CSLIBS_MATH_STABLE_StableDistribution_HPP
#define CSLIBS_MATH_STABLE_StableDistribution_HPP

#include <assert.h>

#include <cslibs_math/approx/sqrt.hpp>
#include <cslibs_math/statistics/limit_eigen_values.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <memory>
#include <mutex>

namespace cslibs_math {
namespace statistics {
template <typename T, std::size_t Dim, std::size_t lambda_ratio_exponent = 0>
class EIGEN_ALIGN16 StableDistribution {
 public:
    static_assert(Dim < static_cast<unsigned int>(std::numeric_limits<int>::max()), "M must be smaller than the maximum signed integer value.");

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using allocator_t = Eigen::aligned_allocator<
      StableDistribution<T, static_cast<int>(Dim), lambda_ratio_exponent>>;

  using Ptr =
      std::shared_ptr<StableDistribution<T, static_cast<int>(Dim), lambda_ratio_exponent>>;
  using sample_t = Eigen::Matrix<T, static_cast<int>(Dim), 1>;
  using sample_transposed_t = Eigen::Matrix<T, 1, static_cast<int>(Dim)>;
  using covariance_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;
  using eigen_values_t = Eigen::Matrix<T, static_cast<int>(Dim), 1>;
  using eigen_vectors_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;

  /// @attention this may cause problems with debug compiles
  static constexpr T sqrt_2_M_PI =
      static_cast<T>(cslibs_math::approx::sqrt(2.0 * M_PI));

  inline StableDistribution() = default;

  inline StableDistribution(const sample_t &mean) : mean_{mean}, n_{1} {}

  inline StableDistribution(std::size_t n, sample_t mean, covariance_t scatter)
      : mean_{mean}, scatter_{scatter}, n_{n} {}

  inline StableDistribution(const StableDistribution &other) = default;

  inline StableDistribution &operator=(const StableDistribution &other) =
      default;

  inline StableDistribution(StableDistribution &&other) = default;

  inline StableDistribution &operator=(StableDistribution &&other) = default;

  inline void reset() {
    mean_ = sample_t::Zero();
    scatter_ = covariance_t::Zero();
    n_ = 0;

    information_matrix_ = covariance_t::Zero();
  }

  /// Modification
  inline void add(const sample_t &p) {
    const sample_t _mean = mean_;
    const std::size_t _n = n_ + 1;
    mean_ = (mean_ * static_cast<T>(n_) + p) / static_cast<T>(_n);
    scatter_ += (p - _mean) * (p - mean_).transpose();
    n_ = _n;

    information_matrix_ = covariance_t::Zero();
  }

  inline void add(const StableDistribution &other) {
    const std::size_t _n = n_ + other.n_;
    const auto dmean = mean_ - other.mean_;
    mean_ =
        (mean_ * static_cast<T>(n_) + other.mean_ * static_cast<T>(other.n_)) /
        static_cast<T>(_n);
    scatter_ += other.scatter_ + static_cast<T>(n_ * other.n_) /
                                     static_cast<T>(_n) * dmean *
                                     dmean.transpose();
    n_ = _n;

    information_matrix_ = covariance_t::Zero();
  }

  inline StableDistribution &operator+=(const sample_t &p) {
    add(p);
    return *this;
  }

  inline StableDistribution &operator+=(const StableDistribution &other) {
    add(other);
    return *this;
  }

  inline bool valid() const { return n_ > Dim; }

  inline std::size_t getN() const { return n_; }

  inline sample_t const &getMean() const { return mean_; }

  inline void getMean(sample_t &_mean) const { _mean = sample_t(mean_); }

  inline covariance_t const &getScatter() const { return scatter_; }

  inline covariance_t getCovariance() const {
    auto update_return_covariance = [this]() {
      update();
      return (n_ > 1 ? covariance_t(scatter_ / static_cast<T>(n_ - 1))
                     : covariance_t::Zero());
    };
    return (dirty() && valid())
               ? update_return_covariance()
               : (n_ > 1 ? covariance_t(scatter_ / static_cast<T>(n_ - 1))
                         : covariance_t::Zero());
  }

  inline void getScatter(covariance_t &s) const { s = covariance_t(scatter_); }

  inline void getCovariance(covariance_t &covariance) const {
    auto update_return_covariance = [this]() {
      update();
      return n_ > 1 ? covariance_t(scatter_ / static_cast<T>(n_ - 1))
                    : covariance_t::Zero();
    };
    covariance = (dirty() && valid())
                     ? update_return_covariance()
                     : (n_ > 1 ? covariance_t(scatter_ / static_cast<T>(n_ - 1))
                               : covariance_t::Zero());
  }

  inline covariance_t getInformationMatrix() const {
    auto update_return_information = [this]() {
      update();
      return information_matrix_;
    };
    return (dirty() && valid()) ? update_return_information()
                                : information_matrix_;
  }

  inline void getInformationMatrix(covariance_t &information_matrix) const {
    auto update_return_information = [this]() {
      update();
      return covariance_t(information_matrix_);
    };
    information_matrix = (dirty() && valid())
                             ? update_return_information()
                             : covariance_t(information_matrix_);
  }

  inline bool getEigenValuesVectors(eigen_values_t &eigen_values,
                                    eigen_vectors_t &eigen_vectors,
                                    const bool abs = false) const {
    auto update_return_eigen = [this, &eigen_values, &eigen_vectors, &abs]() {
      if (dirty()) update();

      Eigen::EigenSolver<covariance_t> solver;
      solver.compute(information_matrix_.inverse().eval());
      eigen_vectors = solver.eigenvectors().real();
      eigen_values =
          abs ? eigen_values_t(solver.eigenvalues().real().cwiseAbs())
              : solver.eigenvalues().real();
      return true;
    };
    return valid() ? update_return_eigen() : false;
  }

  inline T sampleNonNormalized(const sample_t &p) const {
    auto update_sample = [this, &p]() {
      if (dirty()) update();
      const auto &q = p - mean_;
      const T &exponent = -0.5 * q.transpose() * information_matrix_ * q;
      return std::exp(exponent);
    };
    return valid() ? update_sample() : T();  // T() = zero
  }

  inline T sampleNonNormalized(const sample_t &p, sample_t &q) const {
    auto update_sample = [this, &p, &q]() {
      if (dirty()) update();
      q = p - mean_;
      const T exponent = -0.5 * q.transpose() * information_matrix_ * q;
      return std::exp(exponent);
    };
    return valid() ? update_sample() : T();  // T() = zero
  }

  inline T sampleNonNormalizedMean() const {
    return sampleNonNormalized(getMean());
  }

  inline void merge(const StableDistribution &other) { *this += other; }

 private:
  sample_t mean_{sample_t::Zero()};
  covariance_t scatter_{covariance_t::Zero()};
  std::size_t n_{0};

  mutable covariance_t information_matrix_{covariance_t::Zero()};

  inline bool dirty() const { return information_matrix_.isZero(0); }

  inline void update() const {
    const T scale = T(1) / static_cast<T>(n_ - 1);
    information_matrix_ = scale * scatter_;

    information_matrix_ = information_matrix_.inverse().eval();
  }
};

template <typename T, std::size_t lambda_ratio_exponent>
class EIGEN_ALIGN16 StableDistribution<T, 1, lambda_ratio_exponent> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using allocator_t =
      Eigen::aligned_allocator<StableDistribution<T, 1, lambda_ratio_exponent>>;
  using Ptr = std::shared_ptr<StableDistribution<T, 1, lambda_ratio_exponent>>;

  static constexpr T sqrt_2_M_PI = cslibs_math::approx::sqrt(2.0 * M_PI);

  inline StableDistribution() = default;

  inline StableDistribution(std::size_t n, T mean, T scatter)
      : mean_{mean}, scatter_{scatter}, n_{n}, dirty_{true} {}

  inline StableDistribution(const StableDistribution &other) = default;
  inline StableDistribution(StableDistribution &&other) = default;
  inline StableDistribution &operator=(const StableDistribution &other) =
      default;

  inline void reset() {
    mean_ = T();
    scatter_ = T();
    n_ = 0;
    variance_ = T();
    standard_deviation_ = T();
    dirty_ = false;
  }

  inline void add(const T s) {
    const T _mean = mean_;
    const std::size_t _n = n_ + 1;
    mean_ = (mean_ * static_cast<T>(n_) + s) / static_cast<T>(_n);
    scatter_ += (s - _mean) * (s - mean_);
    n_ = _n;
    dirty_ = true;
  }

  inline void add(const StableDistribution &other) {
    const std::size_t _n = n_ + other.n_;
    const auto dmean = mean_ - other.mean_;
    mean_ =
        (mean_ * static_cast<T>(n_) + other.mean_ * static_cast<T>(other.n_)) /
        static_cast<T>(_n);
    scatter_ += other.scatter_ + static_cast<T>(n_ * other.n_) /
                                     static_cast<T>(_n) * dmean * dmean;
    n_ = _n;
    dirty_ = true;
  }

  inline StableDistribution &operator+=(const T s) {
    add(s);
    return *this;
  }

  inline StableDistribution &operator+=(const StableDistribution &other) {
    add(other);
    return *this;
  }

  inline bool valid() const { return n_ > 1; }

  inline std::size_t getN() const { return n_; }

  inline T getMean() const { return mean_; }

  inline T getScatter() const { return scatter_; }

  inline T getVariance() const {
    auto update_return_variance = [this]() {
      update();
      return variance_;
    };
    return (dirty_ && valid()) ? update_return_variance() : variance_;
  }

  inline T getStandardDeviation() const {
    auto update_return_standard_deviation = [this]() {
      update();
      return standard_deviation_;
    };
    return (dirty_ && valid()) ? update_return_standard_deviation()
                               : standard_deviation_;
  }

  inline T sample(const T s) const {
    auto update_sample = [this, &s]() {
      if (dirty_) update();
      const T d = 2.0 * variance_;
      const T x = s - mean_;
      return std::exp(-0.5 * x * x / d) / (sqrt_2_M_PI * standard_deviation_);
    };
    return valid() ? update_sample() : T();
  }

  inline T sampleNonNormalized(const T s) const {
    auto update_sample = [this, &s]() {
      if (dirty_) update();
      const T d = 2.0 * variance_;
      const T x = s - mean_;
      return std::exp(-0.5 * x * x / d);
    };
    return valid() ? update_sample() : T();
  }

  inline void merge(const StableDistribution &other) { *this += other; }

 private:
  T mean_{0};
  T scatter_{0};
  std::size_t n_{0};

  mutable T variance_{0};
  mutable T standard_deviation_{0};

  mutable bool dirty_{false};

  inline void update() const {
    variance_ = scatter_ / static_cast<T>(n_ - 1);
    standard_deviation_ = std::sqrt(variance_);
    dirty_ = false;
  }
};
}  // namespace statistics
}  // namespace cslibs_math

template <typename T, std::size_t D, std::size_t L>
std::ostream &operator<<(
    std::ostream &out,
    const cslibs_math::statistics::StableDistribution<T, D, L> &d) {
  out << d.getMean() << "\n";
  out << d.getCovariance() << "\n";
  out << d.getN() << "\n";
  return out;
}

template <typename T, std::size_t L>
std::ostream &operator<<(
    std::ostream &out,
    const cslibs_math::statistics::StableDistribution<T, 1, L> &d) {
  out << d.getMean() << "\n";
  out << d.getVariance() << "\n";
  out << d.getN() << "\n";
  return out;
}

#endif  // CSLIBS_MATH_STABLE_StableDistribution_HPP
