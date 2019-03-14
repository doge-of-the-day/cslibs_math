#ifndef CSLIBS_MATH_LOG_ODDS_HPP
#define CSLIBS_MATH_LOG_ODDS_HPP

#include <cmath>
#include <cslibs_math/utility/traits.hpp>

namespace cslibs_math {
namespace common {
template <typename T>
struct LogOdds {
    inline static T to(const T p)
    {
        return std::log(p / (utility::traits<T>::One - p));
    }

    inline static T from(const T l)
    {
        return utility::traits<T>::One / (utility::traits<T>::One + std::exp(-l));
    }
};
}
}

#endif // LOG_ODDS_HPP
