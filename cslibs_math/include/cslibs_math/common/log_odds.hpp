#ifndef LOG_ODDS_HPP
#define LOG_ODDS_HPP

#include <cmath>

namespace cslibs_math {
namespace common {
struct LogOdds {
    inline static double to(const double p)
    {
        return std::log(p / (1.0 - p));
    }

    inline static double from(const double l)
    {
        return 1.0 / (1.0 + std::exp(-l));
    }
};
}
}

#endif // LOG_ODDS_HPP
