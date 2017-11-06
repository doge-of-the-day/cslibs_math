#ifndef CSLIBS_MATH_ARRAY_HPP
#define CSLIBS_MATH_ARRAY_HPP

#include <array>
#include <iostream>

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator - (const std::array<_Tp, _Nm> &__one,
                                 const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] - __two[i];
    return arr;
}


template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator + (const std::array<_Tp, _Nm> &__one,
                                 const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] + __two[i];
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator * (const std::array<_Tp, _Nm> &__one,
                                 const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] * __two[i];
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator * (const std::array<_Tp, _Nm> &__one,
                                 const _Tp s)
{
    std::array<_Tp, _Nm> arr = __one;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] *= s;
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator / (const std::array<_Tp, _Nm> &__one,
                                 const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = __one[i] / __two[i];
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> operator / (const std::array<_Tp, _Nm> &__one,
                                 const _Tp s)
{
    std::array<_Tp, _Nm> arr = __one;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] /= s;
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::ostream & operator << (std::ostream &__out, const std::array<_Tp, _Nm> &__arr)
{
    if(_Nm != 0ul) {
        __out << "[";
        for(std::size_t i = 0 ; i < _Nm-1 ; ++i) {
            __out << __arr[i] << ",";
        }
        __out << __arr.back() << "]";
    } else {
        __out << "[]";
    }
    return __out;
}

namespace cslibs_math {
namespace common {
template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> min(const std::array<_Tp, _Nm> &__one,
                         const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = std::min(__one[i], __two[i]);
    return arr;
}

template<typename _Tp, std::size_t _Nm>
std::array<_Tp, _Nm> max(const std::array<_Tp, _Nm> &__one,
                         const std::array<_Tp, _Nm> &__two)
{
    std::array<_Tp, _Nm> arr;
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        arr[i] = std::max(__one[i], __two[i]);
    return arr;
}
}
}

#endif // CSLIBS_MATH_ARRAY_HPP
