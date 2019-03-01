#ifndef CSLIBS_MATH_LOG_ODDS_HPP
#define CSLIBS_MATH_LOG_ODDS_HPP

#include <cmath>

namespace cslibs_math {
namespace common {
template <typename T>
struct LogOdds {
    inline static T to(const T p)
    {
        return std::log(p / static_cast<T>(1.0 - p));
    }

    inline static T from(const T l)
    {
        return static_cast<T>(1.0) / (static_cast<T>(1.0) + std::exp(-l));
    }
};
}
}

#endif // LOG_ODDS_HPP
