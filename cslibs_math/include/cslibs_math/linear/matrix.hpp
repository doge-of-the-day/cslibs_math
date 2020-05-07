#ifndef CSLIBS_MATH_MATRIX_HPP
#define CSLIBS_MATH_MATRIX_HPP

#include <array>
#include <cmath>
#include <cslibs_math/linear/eigen.hpp>
#include <cslibs_math/linear/vector.hpp>
#include <limits>

namespace cslibs_math {
namespace linear {
template <typename T, std::size_t Dim>
class Vector;

template <typename T, std::size_t N, std::size_t M>
class EIGEN_ALIGN16 Matrix {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using allocator_t = Eigen::aligned_allocator<Matrix>;
  using vector_t = Eigen::Matrix<T, N, 1ul>;
  using matrix_t = Eigen::Matrix<T, N, M>;

  using type_t = T;
  const static std::size_t ROWS = N;
  const static std::size_t COLS = M;

  inline Matrix() = default;
  inline Matrix(const T &c) : data_{matrix_t::Constant(c)} {}

  inline Matrix(matrix_t &&data) : data_{data} {};
  inline Matrix(const matrix_t &data) : data_{data} {};

  template <std::size_t m = M, typename std::enable_if<m == 1>::type * = 0>
  inline Matrix(const Vector<T, N> &v) : data_{v.data()} {}

  inline static Matrix identity() { return Matrix{(matrix_t::Identity()).eval()}; }

  inline static Matrix random() { return Matrix{(matrix_t::Random()).eval()}; }

  template <typename... args_t,
            typename = typename std::enable_if<sizeof...(args_t) == N * M>>
  inline explicit Matrix(const args_t... values)
      : data_{eigen::create<matrix_t>(values...)} {}

  inline Matrix(const Matrix &other) = default;

  inline Matrix(Matrix &&other) = default;

  inline Matrix &operator=(const matrix_t &data) {
    data_ = data;
    return *this;
  }

  inline Matrix &operator=(matrix_t &&data) {
    data_ = std::move(data);
    return *this;
  }

  inline Matrix &operator=(const Matrix &other) = default;

  inline Matrix &operator=(Matrix &&other) = default;

  inline Matrix operator*(const T s) const {
    return Matrix{(data_ * s).eval()};
  }

  template <std::size_t X>
  inline Matrix<T, N, X> operator*(const Matrix<T, M, X> &m) const {
    return Matrix<T, N, X>{Eigen::Matrix<T, N, X>{(data_ * m.data()).eval()}};
  }

  inline Matrix operator/(const T s) const {
    return Matrix{(data_ / s).eval()};
  }

  inline Matrix operator+(const Matrix &other) const {
    return Matrix{(data_ + other.data_).eval()};
  }

  inline Matrix operator-(const Matrix &other) const {
    return Matrix{(data_ - other.data_).eval()};
  }

  inline T dot(const Matrix &other) { return data_.dot(other.data_); }

  inline Matrix cross(const Matrix &other) {
    return Matrix{data_.cross(other.data_).eval()};
  }

  inline T norm() const { return data_.norm(); }

  inline T norm2() const { return data_.squaredNorm(); }

  inline T operator()(const std::size_t r, const std::size_t c) const {
    return data_(r, c);
  }

  inline T &operator()(const std::size_t r, const std::size_t c) {
    return data_(r, c);
  }

  inline Matrix &operator+=(const Matrix &other) {
    data_ += other.data_;
    return *this;
  }

  inline Matrix &operator-=(const Matrix &other) {
    data_ -= other.data_;
    return *this;
  }

  inline Matrix &operator*=(const T s) {
    data_ *= s;
    return *this;
  }

  inline Matrix &operator/=(const T s) {
    data_ /= s;
    return *this;
  }

  inline Matrix normalized() const { return Matrix{data_.normalized().eval()}; }

  inline Matrix operator-() const { return Matrix{-data_}; }

  operator const matrix_t &() const { return data_; }

  operator matrix_t &() { return data_; }

  operator matrix_t() const { return data_; }

  operator matrix_t *() { return &data_; }

  inline Matrix min(const Matrix &other) const {
    return Matrix{(data_.cwiseMin(other.data_)).eval()};
  }

  inline Matrix max(const Matrix &other) const {
    return Matrix{(data_.cwiseMax(other.data_)).eval()};
  }

  inline T distance(const Matrix &other) const {
    return (data_ - other.data_).norm();
  }

  inline T distance2(const Matrix &other) const {
    return (data_ - other.data_).squaredNorm();
  }

  inline const matrix_t &data() const { return data_; }

  inline matrix_t &data() { return data_; }

  inline const Matrix<T, M, N> inverse() const {
    return Matrix<T, M, N>{Eigen::Matrix<T, M, N>{(data_.inverse()).eval()}};
  }

  inline Matrix<T, M, N> inverse() {
    return Matrix<T, M, N>{Eigen::Matrix<T, M, N>{(data_.inverse()).eval()}};
  }

  inline Matrix<T, M, N> transpose() {
    return Matrix<T, M, N>{
        static_cast<Eigen::Matrix<T, M, N>>((data_.transpose()).eval())};
  }

  inline Matrix<T, M, N> transpose() const {
    return Matrix<T, M, N>{
        static_cast<Eigen::Matrix<T, M, N>>((data_.transpose()).eval())};
  }

  inline T determinant() { return data_.determinant(); }

  inline T determinant() const { return data_.determinant(); }

 private:
  matrix_t data_{matrix_t::Zero()};
};
}  // namespace linear
}  // namespace cslibs_math
#endif  // CSLIBS_MATH_MATRIX_HPP
