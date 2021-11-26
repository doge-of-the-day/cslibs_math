#ifndef CSLIBS_MATH_STABLE_WEIGHTED_DISTRIBUTION_HPP
#define CSLIBS_MATH_STABLE_WEIGHTED_DISTRIBUTION_HPP

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
class EIGEN_ALIGN16 StableWeightedDistribution {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static_assert(Dim < static_cast<std::size_t>(std::numeric_limits<int>::max()), "'Dim' cannot exceed integer maximum.");

  using allocator_t = Eigen::aligned_allocator<
      StableWeightedDistribution<T, Dim, lambda_ratio_exponent>>;

  using Ptr = std::shared_ptr<
      StableWeightedDistribution<T, Dim, lambda_ratio_exponent>>;
  using sample_t = Eigen::Matrix<T, static_cast<int>(Dim), 1>;
  using sample_transposed_t = Eigen::Matrix<T, 1, static_cast<int>(Dim)>;
  using covariance_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;
  using eigen_values_t = Eigen::Matrix<T, static_cast<int>(Dim), 1>;
  using eigen_vectors_t = Eigen::Matrix<T, static_cast<int>(Dim), static_cast<int>(Dim)>;

  StableWeightedDistribution() = default;

  inline StableWeightedDistribution(std::size_t sample_count, T w, T w_sq,
                                    sample_t mean, covariance_t scatter)
      : sample_count_{sample_count},
        mean_{mean},
        scatter_{scatter},
        W_{w},
        W_sq_{w_sq} {}

  StableWeightedDistribution(const StableWeightedDistribution &other) = default;

  StableWeightedDistribution &operator=(
      const StableWeightedDistribution &other) = default;

  StableWeightedDistribution(StableWeightedDistribution &&other) = default;

  StableWeightedDistribution &operator=(StableWeightedDistribution &&other) =
      default;

  inline void reset() {
    sample_count_ = 0;
    mean_ = sample_t::Zero();
    scatter_ = covariance_t::Zero();
    W_ = T();
    W_sq_ = T();

    information_matrix_ = covariance_t::Zero();
  }

  /// Modification
  inline void add(const sample_t &p, const T w = static_cast<T>(1.0)) {
    if (w <= T()) return;

    const T _W = W_ + w;
    const auto mean = mean_;
    mean_ = (mean_ * W_ + p * w) / _W;
    scatter_ += w * (p - mean) * (p - mean_).transpose();
    W_ = _W;
    W_sq_ += w * w;
    ++sample_count_;

    information_matrix_ = covariance_t::Zero();
  }

  inline StableWeightedDistribution &operator+=(
      const StableWeightedDistribution &other) {
    const T _W = W_ + other.W_;
    const auto dmean = mean_ - other.mean_;
    mean_ = (mean_ * W_ + other.mean_ * other.W_) / _W;
    scatter_ +=
        other.scatter_ + (W_ * other.W_) / _W * dmean * dmean.transpose();
    W_ = _W;
    W_sq_ += other.W_sq_;
    sample_count_ += other.sample_count_;

    information_matrix_ = covariance_t::Zero();
    return *this;
  }

  /// Distribution properties
  inline bool valid() const { return sample_count_ > Dim; }

  inline std::size_t getSampleCount() const { return sample_count_; }

  inline T getWeight() const { return W_; }

  inline T getWeightSQ() const { return W_sq_; }

  inline sample_t getMean() const { return mean_; }

  inline void getMean(sample_t &mean) const { mean = sample_t(mean_); }

  inline covariance_t getScatter() const { return scatter_; }

  inline covariance_t getCovariance() const {
    auto update_return_covariance = [this]() {
      update();
      return information_matrix_.inverse();
    };
    return (dirty() && valid()) ? update_return_covariance()
                                : information_matrix_.inverse();
  }

  inline void getCovariance(covariance_t &covariance) const {
    auto update_return_covariance = [this]() {
      update();
      return covariance_t(information_matrix_.inverse());
    };
    covariance = (dirty() && valid())
                     ? update_return_covariance()
                     : covariance_t(information_matrix_.inverse());
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
      solver.compute(information_matrix_.inverse());
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
      const sample_t q = p - mean_;
      const T exponent =
          -0.5 *
          static_cast<T>(static_cast<sample_transposed_t>(q.transpose()) *
                         information_matrix_ * q);
      return std::exp(exponent);
    };
    return valid() ? update_sample() : T();
  }

  inline T sampleNonNormalized(const sample_t &p, sample_t &q) const {
    auto update_sample = [this, &p, &q]() {
      if (dirty()) update();
      q = p - mean_;
      const T exponent =
          -0.5 *
          static_cast<T>(static_cast<sample_transposed_t>(q.transpose()) *
                         information_matrix_ * q);
      return std::exp(exponent);
    };
    return valid() ? update_sample() : T();
  }

  inline void merge(const StableWeightedDistribution &other) { *this += other; }

 private:
  std::size_t sample_count_{0};
  sample_t mean_{sample_t::Zero()};
  covariance_t scatter_{covariance_t::Zero()};
  T W_{0};
  T W_sq_{0};
  mutable covariance_t information_matrix_{covariance_t::Zero()};

  inline bool dirty() const { return information_matrix_.isZero(0); }

  inline void update() const {
    const T scale = T(1.0) / (W_ - W_sq_ / W_);
    information_matrix_ = scale * scatter_;
    information_matrix_ = information_matrix_.inverse().eval();
  }
};

template <typename T, std::size_t lambda_ratio_exponent>
class EIGEN_ALIGN16 StableWeightedDistribution<T, 1, lambda_ratio_exponent> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using allocator_t = Eigen::aligned_allocator<
      StableWeightedDistribution<T, 1, lambda_ratio_exponent>>;
  using Ptr =
      std::shared_ptr<StableWeightedDistribution<T, 1, lambda_ratio_exponent>>;

  static constexpr T sqrt_2_M_PI = cslibs_math::approx::sqrt(2.0 * M_PI);

  inline StableWeightedDistribution() = default;

  inline StableWeightedDistribution(std::size_t sample_count, T w, T w_sq,
                                    T mean, T scatter)
      : sample_count_{sample_count},
        mean_{mean},
        scatter_{scatter},
        W_{w},
        W_sq_{w_sq},
        dirty_{true} {}

  inline StableWeightedDistribution(const StableWeightedDistribution &other) =
      default;
  inline StableWeightedDistribution(StableWeightedDistribution &&other) =
      default;
  inline StableWeightedDistribution &operator=(
      const StableWeightedDistribution &other) = default;

  inline void reset() {
    sample_count_ = 0;
    mean_ = T();
    scatter_ = T();
    variance_ = T();
    W_ = T();
    W_sq_ = T();
    standard_deviation_ = T();
    dirty_ = false;
  }

  inline void add(const T s, const T w) {
    const T _W = W_ + w;
    const auto mean = mean_;
    mean_ = (mean_ * W_ + s * w) / _W;
    scatter_ += w * (s - mean) * (s - mean_);
    W_ = _W;
    W_sq_ += w * w;
    ++sample_count_;
    dirty_ = true;
  }

  inline StableWeightedDistribution &operator+=(
      const StableWeightedDistribution &other) {
    const T _W = W_ + other.W_;
    const auto dmean = mean_ - other.mean_;
    mean_ = (mean_ * W_ + other.mean_ * other.W_) / _W;
    scatter_ += other.scatter_ + (W_ * other.W_) / _W * dmean * dmean;
    W_ = _W;
    W_sq_ += other.W_sq_;
    sample_count_ += other.sample_count_;
    dirty_ = true;
    return *this;
  }

  inline bool valid() const { return sample_count_ > 1; }

  inline std::size_t getSampleCount() const { return sample_count_; }

  inline T getWeight() const { return W_; }

  inline T getWeightSQ() const { return W_sq_; }

  inline T getMean() const { return mean_; }

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

  inline void merge(const StableWeightedDistribution &other) { *this += other; }

 private:
  std::size_t sample_count_{0};
  T mean_{0};
  T scatter_{0};
  T W_{0};
  T W_sq_{0};

  mutable T variance_{0};
  mutable T standard_deviation_{0};

  mutable bool dirty_{false};

  inline void update() const {
    const T scale = T(1) / (W_ - W_sq_ / W_);
    variance_ = scatter_ * scale;  //(squared_ - mean_ * mean_) * scale;
    standard_deviation_ = std::sqrt(variance_);
    dirty_ = false;
  }
};
}  // namespace statistics
}  // namespace cslibs_math

template <typename T, std::size_t D, std::size_t L>
std::ostream &operator<<(
    std::ostream &out,
    const cslibs_math::statistics::StableWeightedDistribution<T, D, L> &d) {
  out << d.getMean() << "\n";
  out << d.getCovariance() << "\n";
  out << d.getWeight() << "\n";
  return out;
}

template <typename T, std::size_t L>
std::ostream &operator<<(
    std::ostream &out,
    const cslibs_math::statistics::StableWeightedDistribution<T, 1, L> &d) {
  out << d.getMean() << "\n";
  out << d.getVariance() << "\n";
  out << d.getWeight() << "\n";
  return out;
}

#endif  // CSLIBS_MATH_STABLE_WEIGHTED_DISTRIBUTION_HPP
