#ifndef CSLIBS_MATH_LOG_ODDS_HPP
#define CSLIBS_MATH_LOG_ODDS_HPP

#include <cmath>
#include <ros/console.h>

namespace cslibs_math {
namespace common {

template <typename T>
struct LogOdds {
    static inline T to(const T p)
    {
        return std::log(p / (1.0 - p));
    }

    static inline T from(const T l)
    {
        return 1.0 / (1.0 + std::exp(-l));
    }
};

}
}

#endif // CSLIBS_MATH_LOG_ODDS_HPP
