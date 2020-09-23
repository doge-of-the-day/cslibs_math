#ifndef CSLIBS_MATH_3D_NDT_ITERATOR_HPP
#define CSLIBS_MATH_3D_NDT_ITERATOR_HPP

#include <memory>

#include <cslibs_math/common/div.hpp>
#include <cslibs_math/common/mod.hpp>
#include <cslibs_math/common/array.hpp>
#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
namespace algorithms {
template <typename T>
class EIGEN_ALIGN16 NDTIterator
{
public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr           = std::shared_ptr<NDTIterator<T>>;

    using index_t       = std::array<int, 3>;
    using error_t       = std::array<T, 2>;
    using point_t       = Point3<T>;

    inline explicit NDTIterator(const index_t &start,
                                const index_t &end) :
        NDTIterator(point_t(start[0], start[1], start[2]),
                    point_t(end[0], end[1], end[2]), 1.0)
    {
    }

    inline explicit NDTIterator(const point_t &p0,
                                const point_t &p1,
                                const T  &resolution) :
        index_({{static_cast<int>(std::floor(p0(0) / resolution)),
                 static_cast<int>(std::floor(p0(1) / resolution)),
                 static_cast<int>(std::floor(p0(2) / resolution))}})
    {
        const index_t end({{static_cast<int>(std::floor(p1(0) / resolution)),
                            static_cast<int>(std::floor(p1(1) / resolution)),
                            static_cast<int>(std::floor(p1(2) / resolution))}});

        // mod function
        auto mod = [&resolution](const T& x) {
          const T& m = std::fmod(x/resolution, T(1.));
          return (m >= T(0.) ? m : (m+T(1.)));
        };

        // Bresenham delta and steps
        step_      = std::compare(index_, end);
        const T dx = std::fabs(p1(0) - p0(0));
        const T dy = std::fabs(p1(1) - p0(1));
        const T dz = std::fabs(p1(2) - p0(2));

        // calculate initial error, set dominant direction and
        // execute one step into target direction
        if (dx > dy && dx > dz) {
            error_inc_[0] = dy/dx;
            error_inc_[1] = dz/dx;
            error_[0]  = (0.5 - mod(p0(0))) * error_inc_[0] * step_[1] + mod(p0(1)) - (step_[1] > 0) ? 1 : 0;
            error_[1]  = (0.5 - mod(p0(0))) * error_inc_[1] * step_[2] + mod(p0(2)) - (step_[2] > 0) ? 1 : 0;

            iterate_   = &NDTIterator::iterateDx;
            (this->*iterate_)();
            iteration_ = (std::abs(end[0] - index_[0]) - 1) >> 1;

            error_inc_[0] *= 2;
            error_inc_[1] *= 2;
            step_[0]   *= 2;
        } else if (dy > dx && dy > dz) {
            error_inc_[0] = dx/dy;
            error_inc_[1] = dz/dy;
            error_[0]  = (0.5 - mod(p0(1))) * error_inc_[0] * step_[0] + mod(p0(0)) - (step_[0] > 0) ? 1 : 0;
            error_[1]  = (0.5 - mod(p0(1))) * error_inc_[1] * step_[2] + mod(p0(2)) - (step_[2] > 0) ? 1 : 0;

            iterate_   = &NDTIterator::iterateDy;
            (this->*iterate_)();
            iteration_ = (std::abs(end[1] - index_[1]) - 1) >> 1;

            error_inc_[0] *= 2;
            error_inc_[1] *= 2;
            step_[1]   *= 2;
        } else {
            error_inc_[0] = dx/dz;
            error_inc_[1] = dy/dz;
            error_[0]  = (0.5 - mod(p0(2))) * error_inc_[0] * step_[0] + mod(p0(0)) - (step_[0] > 0) ? 1 : 0;
            error_[1]  = (0.5 - mod(p0(2))) * error_inc_[1] * step_[1] + mod(p0(1)) - (step_[1] > 0) ? 1 : 0;

            iterate_   = &NDTIterator::iterateDz;
            (this->*iterate_)();
            iteration_ = (std::abs(end[2] - index_[2]) - 1) >> 1;

            error_inc_[0] *= 2;
            error_inc_[1] *= 2;
            step_[2]   *= 2;
        }
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

    inline index_t operator()() const
    {
        return index_;
    }

    inline NDTIterator& operator++()
    {
        return done() ? *this : (this->*iterate_)();
    }

    inline bool done() const
    {
        return (iteration_ <= 0);
    }

private:
    inline NDTIterator &iterateDx()
    {
        error_[0] += error_inc_[0];
        error_[1] += error_inc_[1];
        index_[0] += step_[0];
        while (error_[0] > 0) {
            index_[1] += step_[1];
            --error_[0];
        }
        while (error_[1] > 0) {
            index_[2] += step_[2];
            --error_[1];
        }
        --iteration_;
        return *this;
    }

    inline NDTIterator &iterateDy()
    {
        error_[0] += error_inc_[0];
        error_[1] += error_inc_[1];
        index_[1] += step_[1];
        while (error_[0] > 0) {
            index_[0] += step_[0];
            --error_[0];
        }
        while (error_[1] > 0) {
            index_[2] += step_[2];
            --error_[1];
        }
        --iteration_;
        return *this;
    }

    inline NDTIterator &iterateDz()
    {
        error_[0] += error_inc_[0];
        error_[1] += error_inc_[1];
        index_[2] += step_[2];
        while (error_[0] > 0) {
            index_[0] += step_[0];
            --error_[0];
        }
        while (error_[1] > 0) {
            index_[1] += step_[1];
            --error_[1];
        }
        --iteration_;
        return *this;
    }

    index_t index_;
    index_t step_;
    error_t error_;
    error_t error_inc_;
    int     iteration_;

    NDTIterator& (NDTIterator::*iterate_)();
};
}
}

#endif // CSLIBS_MATH_3D_NDT_ITERATOR_HPP
