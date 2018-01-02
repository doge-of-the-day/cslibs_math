#ifndef CSLIBS_MATH_3D_AMANTIDIS_HPP
#define CSLIBS_MATH_3D_AMANTIDIS_HPP

#include <memory>
#include <array>
#include <cmath>
#include <cslibs_math/common/equal.hpp>

#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
namespace algorithms {
class Amantidis
{
public:
    using Ptr           = std::shared_ptr<Amantidis>;
    using index_t       = std::array<int, 3>;
    using delta_t       = std::array<double, 3>;
    using delta_mask_t  = std::array<bool, 3>;
    using point_t       = Point3d;

    inline Amantidis() :
        start_{{0, 0, 0}},
        end_{{0, 0, 0}},
        index_{{0, 0, 0}},
        delta_{{0.0, 0.0, 0.0}},
        max_{{0.0, 0.0, 0.0}}
    {
    }

    inline explicit Amantidis(const point_t     &start,
                              const point_t     &end,
                              const double      resolution) :
        start_{{static_cast<int>(start(0) / resolution),
                static_cast<int>(start(1) / resolution),
                static_cast<int>(start(2) / resolution)}},
        end_{{static_cast<int>(end(0) / resolution),
              static_cast<int>(end(1) / resolution),
              static_cast<int>(end(2) / resolution)}},
        index_(start_),
        delta_{{0.0, 0.0, 0.0}},
        max_{{0.0, 0.0, 0.0}}
    {
        const point_t d = end - start;
        const static double dmax =  std::numeric_limits<double>::max();
        const bool dx = cslibs_math::common::neq(d(0), 0.0);
        const bool dy = cslibs_math::common::neq(d(1), 0.0);
        const bool dz = cslibs_math::common::neq(d(2), 0.0);
        step_[0]       = d(0) >= 0.0 ? 1 : -1;
        step_[1]       = d(1) >= 0.0 ? 1 : -1;
        step_[2]       = d(2) >= 0.0 ? 1 : -1;
        delta_[0]      = dx ? resolution / d(0) * step_[0] : dmax;
        delta_[1]      = dy ? resolution / d(1) * step_[1] : dmax;
        delta_[2]      = dz ? resolution / d(2) * step_[2] : dmax;
        max_[0]        = dx ? ((start_[0] + step_[0]) * resolution - start(0)) / d(0) : dmax;
        max_[1]        = dy ? ((start_[1] + step_[1]) * resolution - start(1)) / d(1) : dmax;
        max_[2]        = dz ? ((start_[2] + step_[2]) * resolution - start(2)) / d(2) : dmax;
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

    inline int z() const
    {
        return index_[2];
    }

    inline Amantidis& operator++()
    {
        std::size_t dim =  max_[0] < max_[1] ?
                          (max_[0] < max_[2] ? 0ul : 2ul) :
                          (max_[1] < max_[2] ? 1ul : 2ul) ;

        return done() ? *this : iterate(dim);
    }

    inline int length2() const
    {
        auto sq = [](const int d) { return d*d;};
        return sq(index_[0] - end_[0]) +
               sq(index_[1] - end_[1]) +
               sq(index_[2] - end_[2]);
    }

    inline bool done() const
    {
        return index_[0] == end_[0] &&
               index_[1] == end_[1] &&
               index_[2] == end_[2];
    }

private:
    inline Amantidis &iterate(const std::size_t dim)
    {
        max_[dim]   += delta_[dim];
        index_[dim] += step_[dim];
        return *this;
    }

    index_t start_;
    index_t end_;
    index_t index_;
    index_t step_;
    delta_t delta_;
    delta_t max_;
};
}
}

#endif // CSLIBS_MATH_3D_AMANTIDIS_HPP
