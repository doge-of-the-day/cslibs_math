#ifndef CSLIBS_MATH_MEAN_HPP
#define CSLIBS_MATH_MEAN_HPP

#include <eigen3/Eigen/Core>

namespace cslibs_math {
namespace statistics {
template <typename T, std::size_t Dim>
class EIGEN_ALIGN16 Mean {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using allocator_t = Eigen::aligned_allocator<Mean<T, Dim>>;

  using Ptr = std::shared_ptr<Mean<T, Dim>>;
  using sample_t = Eigen::Matrix<T, Dim, 1>;

  inline Mean() = default;
  inline Mean(const Mean &other) = default;
  inline Mean(Mean &&other) = default;

  inline void add(const sample_t &sample) {
    const std::size_t _n = n_ + 1;
    mean_ = (mean_ * static_cast<T>(n_) + sample) / static_cast<T>(_n);
    n_ = _n;
  }

  inline sample_t const &get() const { return mean_; }
  inline std::size_t getN() const { return n_; }

 private:
  sample_t mean_{sample_t::Zero()};
  std::size_t n_{0};
};

template <typename T>
class EIGEN_ALIGN16 Mean<T, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using allocator_t = Eigen::aligned_allocator<Mean<T, 1>>;

  inline void add(const T &sample) {
    const std::size_t _n = n_ + 1;
    mean_ = (mean_ * static_cast<T>(n_) + sample) / static_cast<T>(_n);
    n_ = _n;
  }

  inline T get() const { return mean_; }

  inline std::size_t getN() const { return n_; }

 private:
  T mean_{0};
  std::size_t n_{0};
};
}  // namespace statistics
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_MEAN_HPP
