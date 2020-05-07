#ifndef CSLIBS_MATH_STAMPED_HPP
#define CSLIBS_MATH_STAMPED_HPP

#include <eigen3/Eigen/Core>
#include <memory>

#include "tiny_time.hpp"

namespace cslibs_math {
namespace utility {
template <typename T>
class EIGEN_ALIGN16 Stamped {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using allocator_t = Eigen::aligned_allocator<Stamped<T>>;

  using Ptr = std::shared_ptr<Stamped<T>>;
  using ConstPtr = std::shared_ptr<const Stamped<T>>;
  using time_t = tiny_time::time_t;

  explicit inline Stamped(const time_t &time) : time_{time} {}

  inline explicit Stamped(const T &data, const time_t &time)
      : data_{data}, time_{time} {}

  inline Stamped(const Stamped<T> &other) = default;

  inline Stamped(Stamped<T> &&other) = default;
  inline Stamped<T> &operator=(const Stamped<T> &other) = default;
  inline Stamped<T> &operator=(Stamped<T> &&other) = default;

  inline time_t &stamp() { return time_; }

  inline time_t const &stamp() const { return time_; }

  inline T &data() { return data_; }

  inline T const &data() const { return data_; }

  inline operator T() { return data_; }

  inline operator T &() { return data_; }

  inline operator T *() { return &data_; }

  inline operator const T &() const { return data_; }

  inline operator T() const { return data_; }

 private:
  T data_;
  time_t time_;
};
}  // namespace utility
}  // namespace cslibs_math

#endif  // CSLIBS_MATH_STAMPED_HPP
