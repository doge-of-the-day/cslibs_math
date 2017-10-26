#ifndef ARRAY_HPP
#define ARRAY_HPP

#include <array>

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
    _Tp s = _Tp(0);
    for(std::size_t i = 0 ; i < _Nm ; ++i)
        s += __one[i] * __two[i];
    return s;
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

#endif // ARRAY_HPP
