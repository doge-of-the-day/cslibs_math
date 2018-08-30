#ifndef CSLIBS_MATH_TINY_TIME_HPP
#define CSLIBS_MATH_TINY_TIME_HPP

#include <chrono>

namespace cslibs_math {
namespace utility {
namespace tiny_time {
using clock_t    = std::chrono::high_resolution_clock;
using time_t     = clock_t::time_point;
using duration_t = clock_t::duration;

inline double seconds(const time_t &time)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count() * 1e-9;
}

inline double seconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() * 1e-9;
}

inline double milliseconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() * 1e-6;
}

inline double microseconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() * 1e-3;
}

inline int64_t nanoseconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}
}
}
}

#endif // CSLIBS_MATH_TINY_TIME_HPP
