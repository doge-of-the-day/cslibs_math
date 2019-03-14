#ifndef CSLIBS_MATH_2D_AMANTIDIS_HPP
#define CSLIBS_MATH_2D_AMANTIDIS_HPP

#include <memory>
#include <array>
#include <cmath>
#include <cslibs_math/common/equal.hpp>

#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
namespace algorithms {
template <typename Tp = double>
class EIGEN_ALIGN16 Amantidis
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr           = std::shared_ptr<Amantidis<Tp>>;

    using index_t       = std::array<int, 2>;
    using delta_t       = std::array<Tp, 2>;
    using delta_mask_t  = std::array<bool, 2>;

    inline Amantidis() :
        start_{{0,0}},
        end_{{0,0}},
        index_{{0,0}},
        delta_{{Tp(), Tp()}},
        max_{{Tp(), Tp()}}
    {
    }

    template <typename T>
    inline explicit Amantidis(const Point2<T> &start,
                              const Point2<T> &end,
                              const T           resolution) :
        start_{{static_cast<int>(std::floor(start(0) / resolution)),
                static_cast<int>(std::floor(start(1) / resolution))}},
        end_{{static_cast<int>(std::floor(end(0) / resolution)),
              static_cast<int>(std::floor(end(1) / resolution))}},
        index_(start_),
        delta_{{Tp(), Tp()}},
        max_{{Tp(), Tp()}}
    {
        const Point2<T> d = end - start;
        const static T dmax = std::numeric_limits<T>::max();
        const bool dx = cslibs_math::common::neq(d(0), Tp());
        const bool dy = cslibs_math::common::neq(d(1), Tp());
        step_[0]       = d(0) >= Tp() ? 1 : -1;
        step_[1]       = d(1) >= Tp() ? 1 : -1;
        delta_[0]      = dx ? resolution / d(0) * step_[0] : dmax;
        delta_[1]      = dy ? resolution / d(1) * step_[1] : dmax;
        max_[0]        = dx ? ((start_[0] + step_[0]) * resolution - start(0)) / d(0) : dmax;
        max_[1]        = dy ? ((start_[1] + step_[1]) * resolution - start(1)) / d(1) : dmax;
    }

    inline virtual ~Amantidis()
    {
    }

    inline int x() const
    {
        return index_[0];
    }

    inline int y() const
    {
        return index_[1];
    }

    inline index_t operator()() const
    {
        return index_;
    }

    inline Amantidis& operator++()
    {
        return done() ? *this : iterate(max_[0] < max_[1] ? 0ul : 1ul);
    }

    inline int length2() const
    {
        auto sq = [](const int d) { return d*d;};
        return sq(index_[0] - end_[0]) + sq(index_[1] - end_[1]);
    }

    inline bool done() const
    {
        return index_[0] == end_[0] && index_[1] == end_[1];
    }

private:
    inline Amantidis &iterate(const std::size_t dim)
    {
        max_[dim]   += delta_[dim];
        index_[dim] += step_[dim];
        return *this;
    }

    index_t     start_;
    index_t     end_;
    index_t     index_;
    index_t     step_;
    delta_t     delta_;
    delta_t     max_;
};
}
}

#endif // CSLIBS_MATH_2D_AMANTIDIS_HPP
