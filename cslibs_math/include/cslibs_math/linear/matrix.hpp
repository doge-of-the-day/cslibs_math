#ifndef CSLIBS_MATH_MATRIX_HPP
#define CSLIBS_MATH_MATRIX_HPP

#include <array>
#include <cmath>
#include <limits>

#include <cslibs_math/linear/eigen.hpp>
#include <cslibs_math/linear/vector.hpp>

namespace cslibs_math {
namespace linear {
template<typename T, std::size_t N, std::size_t M>
class Matrix {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using vector_t = Eigen::Matrix<T, N, 1>;
    using matrix_t = Eigen::Matrix<T, N, M>;

    inline Matrix() :
        data_(matrix_t::Zero())
    {
    }

    inline Matrix(const T &c) :
        data_(matrix_t::Constant(c))
    {
    }

    inline Matrix(matrix_t && data) :
        data_(std::move(data))
    {
    }

    inline Matrix(const matrix_t & data) :
        data_(data)
    {
    }

    template<typename... args_t, typename = typename std::enable_if<sizeof...(args_t) == N*M>>
    inline explicit Matrix(const args_t ... values) :
        data_(eigen::create<matrix_t>(values...))
    {
    }

    inline Matrix(const Matrix &other) :
        data_(other.data_)
    {
    }

    inline Matrix(Matrix &&other) :
        data_(std::move(other.data_))
    {
    }

    inline Matrix& operator = (const matrix_t &data)
    {
        data_ = data;
        return *this;
    }

    inline Matrix& operator = (matrix_t && data)
    {
        data_ = std::move(data);
        return *this;
    }

    inline Matrix& operator = (const Matrix &other)
    {
        data_ = other.data_;
        return *this;
    }

    inline Matrix& operator = (Matrix && other)
    {
        data_ = std::move(other.data_);
        return *this;
    }

    inline Matrix operator * (const T s) const
    {
        return Matrix(matrix_t(data_ * s));
    }

    inline vector_t operator * (const vector_t &v) const
    {
        return vector_t(data_ * v);
    }

    inline Matrix operator / (const T s) const
    {
        return Matrix(matrix_t(data_ / s));
    }

    inline Matrix operator + (const Matrix &other) const
    {
        return Matrix(matrix_t(data_ + other.data_));
    }

    inline Matrix operator - (const Matrix &other) const
    {
        return Matrix(matrix_t(data_ - other.data_));
    }

    inline T dot(const Matrix &other)
    {
        return data_.dot(other.data_);
    }

    inline Matrix cross(const Matrix &other)
    {
        return Matrix(data_.cross(other.data_));
    }

    inline T norm() const
    {
        return data_.norm();
    }

    inline T norm2() const
    {
        return data_.squaredNorm();
    }

    inline T operator ()(const std::size_t r, const std::size_t c) const
    {
        return data_(r,c);
    }

    inline T& operator()(const std::size_t r, const std::size_t c)
    {
        return data_(r,c);
    }

    inline Matrix & operator += (const Matrix &other)
    {
        data_ += other.data_;
        return *this;
    }

    inline Matrix & operator -= (const Matrix &other)
    {
        data_ -= other.data_;
        return *this;
    }

    inline Matrix & operator *= (const T s)
    {
        data_ *= s;
        return *this;
    }

    inline Matrix & operator /= (const double s)
    {
        data_ /= s;
        return *this;
    }

    inline Matrix normalized() const
    {
        return Matrix(data_.normalized());
    }

    inline Matrix operator - () const
    {
        return Matrix(-data_);
    }

    operator const matrix_t& () const
    {
        return data_;
    }

    operator matrix_t& ()
    {
        return data_;
    }

    operator matrix_t () const
    {
        return data_;
    }

    operator matrix_t* ()
    {
        return &data_;
    }

    inline Matrix min(const Matrix &other) const
    {
        return Matrix(matrix_t(data_.cwiseMin(other.data_)));
    }

    inline Matrix max(const Matrix &other) const
    {
        return Matrix(matrix_t(data_.cwiseMax(other.data_)));
    }

    inline T distance(const Matrix &other) const
    {
        return (data_ - other.data_).norm();
    }

    inline T distance2(const Matrix &other) const
    {
        return (data_ - other.data_).squaredNorm();
    }

    inline const matrix_t& data() const
    {
        return data_;
    }

    inline matrix_t& data()
    {
        return data_;
    }

private:
    matrix_t data_;
};
}
}
#endif // CSLIBS_MATH_MATRIX_HPP
