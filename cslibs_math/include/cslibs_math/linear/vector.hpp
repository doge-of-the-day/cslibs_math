#ifndef CSLIBS_MATH_VECTOR_HPP
#define CSLIBS_MATH_VECTOR_HPP

#include <array>
#include <cmath>
#include <limits>

#include <eigen3/Eigen/Dense>
#include <cslibs_math/linear/eigen.hpp>

namespace cslibs_math {
namespace linear {
template<typename T, std::size_t Dim>
class Vector {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using vector_t = Eigen::Matrix<T, Dim, 1>;

    using type_t = T;
    const static std::size_t size = Dim;

    inline Vector() :
        data_(vector_t::Zero())
    {
    }

    inline explicit Vector(const T &c) :
        data_(vector_t::Constant(c))
    {
    }

    inline explicit Vector(vector_t && data) :
        data_(std::move(data))
    {
    }

    inline explicit Vector(const vector_t & data) :
        data_(data)
    {
    }

    template<typename... args_t, typename = typename std::enable_if<sizeof...(args_t) == Dim>>
    inline explicit Vector(const args_t ... values) :
        data_(eigen::create<vector_t>(values...))
    {
    }

    inline Vector(const Vector &other) :
        data_(other.data_)
    {
    }

    inline Vector(Vector &&other) :
        data_(std::move(other.data_))
    {
    }

    inline Vector& operator = (const vector_t &data)
    {
        data_ = data;
        return *this;
    }

    inline Vector& operator = (vector_t && data)
    {
        data_ = std::move(data);
        return *this;
    }

    inline Vector& operator = (const Vector &other)
    {
        data_ = other.data_;
        return *this;
    }

    inline Vector& operator = (Vector && other)
    {
        data_ = std::move(other.data_);
        return *this;
    }

    inline Vector operator * (const T s) const
    {
        return Vector(vector_t(data_ * s));
    }

    inline Vector operator / (const T s) const
    {
        return Vector(vector_t(data_ / s));
    }

    inline Vector operator + (const Vector &other) const
    {
        return Vector(vector_t(data_ + other.data_));
    }

    inline Vector operator - (const Vector &other) const
    {
        return Vector(vector_t(data_ - other.data_));
    }

    inline T dot(const Vector &other)
    {
        return data_.dot(other.data_);
    }

    inline Vector cross(const Vector &other)
    {
        return Vector(data_.cross(other.data_));
    }

    inline T length() const
    {
        return data_.norm();
    }

    inline T length2() const
    {
        return data_.squaredNorm();
    }

    inline T operator ()(const std::size_t i) const
    {
        return data_(i);
    }

    inline T& operator()(const std::size_t i)
    {
        return data_(i);
    }

    inline Vector & operator += (const Vector &other)
    {
        data_ += other.data_;
        return *this;
    }

    inline Vector & operator -= (const Vector &other)
    {
        data_ -= other.data_;
        return *this;
    }

    inline Vector & operator *= (const T s)
    {
        data_ *= s;
        return *this;
    }

    inline Vector & operator /= (const double s)
    {
        data_ /= s;
        return *this;
    }

    inline Vector normalized() const
    {
        return Vector(data_.normalized());
    }

    inline Vector operator - () const
    {
        return Vector(-data_);
    }

    operator const vector_t& () const
    {
        return data_;
    }

    operator vector_t& ()
    {
        return data_;
    }

    operator vector_t () const
    {
        return data_;
    }

    operator vector_t* ()
    {
        return &data_;
    }

    inline Vector min(const Vector &other) const
    {
        return Vector(vector_t(data_.cwiseMin(other.data_)));
    }

    inline Vector max(const Vector &other) const
    {
        return Vector(vector_t(data_.cwiseMax(other.data_)));
    }

    inline T distance(const Vector &other) const
    {
        return (data_ - other.data_).norm();
    }

    inline T distance2(const Vector &other) const
    {
        return (data_ - other.data_).squaredNorm();
    }

    inline const vector_t& data() const
    {
        return data_;
    }

    inline vector_t& data()
    {
        return data_;
    }

    inline bool isNormal() const
    {
        return eigen::isnormal(data_);
    }

    inline T angle(const Vector &other)
    {
        return std::acos(dot(other) / (length() * other.length()));
    }

    inline static Vector random()
    {
        Vector v;
        v.data_ = vector_t::Random();
        return v;
    }

private:
    vector_t data_;
};
}
}

#endif // CSLIBS_MATH_VECTOR_HPP
