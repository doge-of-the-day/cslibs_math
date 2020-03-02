#ifndef CSLIBS_MATH_2D_NDT_ITERATOR_HPP
#define CSLIBS_MATH_2D_NDT_ITERATOR_HPP

#include <memory>
#include <cslibs_math/common/array.hpp>
#include <cslibs_math/common/floor.hpp>
#include <cslibs_math/common/div.hpp>
#include <cslibs_math/common/mod.hpp>
#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
namespace algorithms {
template <typename T>
class EIGEN_ALIGN16 NDTIterator
{
public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr           = std::shared_ptr<NDTIterator<T>>;

    using index_t       = std::array<int, 2>;
    using point_t       = Point2<T>;

    inline explicit NDTIterator(const index_t &start,
                                const index_t &end) :
        NDTIterator(point_t(start[0], start[1]),
                    point_t(end[0], end[1]), 1.0)
    {
    }

    inline explicit NDTIterator(const point_t &p0,
                                const point_t &p1,
                                const T  &resolution)  :
        index_({{static_cast<int>(std::floor(p0(0) / resolution)),
                 static_cast<int>(std::floor(p0(1) / resolution))}})
    {
        const index_t end({{static_cast<int>(std::floor(p1(0) / resolution)),
                            static_cast<int>(std::floor(p1(1) / resolution))}});

        // estimate min/max
        auto minmax = [this,end](const std::size_t &i) {
            const auto& p = std::minmax(index_[i],end[i]);
            min_[i] = p.first;
            max_[i] = p.second;
        };
        for (std::size_t i=0; i<2; ++i)
            minmax(i);

        // Bresenham delta and steps
        step_      = std::compare(index_, end);
        const T dx = std::fabs(p1(0) - p0(0));
        const T dy = std::fabs(p1(1) - p0(1));

        // calculate initial error, set dominant direction and
        // execute one step into target direction
        if (dx > dy) {
            error_inc_ = dy/dx;
            error_     = (0.5 - std::fmod(p0(0),resolution)) * error_inc_
                         + std::fmod(p0(1),resolution) - 0.5;
            error_    += (step_[1] > 0) ? -0.5 : 0.5;

            iterate_   = &NDTIterator::iterateDx;
            iteration_ = (std::abs(end[0] - index_[0]) - 2) >> 1;

            min_[0] = std::min(index_[0],end[0]-1);
            max_[0] = std::max(index_[0],end[0]-1);
        } else {
            error_inc_ = dx/dy;
            error_     = (0.5 - std::fmod(p0(1),resolution)) * error_inc_
                         + std::fmod(p0(0),resolution) - 0.5;
            error_    += (step_[0] > 0) ? -0.5 : 0.5;

            iterate_   = &NDTIterator::iterateDy;
            iteration_ = (std::abs(end[1] - index_[1]) - 2) >> 1;

            min_[1] = std::min(index_[1],end[1]-1);
            max_[1] = std::max(index_[1],end[1]-1);
        }

        // two steps at once
        step_[0] *= 2;
        step_[1] *= 2;

        // shift midpoint rule to center of next pixel
        // divide error by two (or multiply error incs by two)
        error_ -= 0.5;
        error_ /= 2;
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

    inline NDTIterator& operator++()
    {
        return done() ? *this : (this->*iterate_)();
    }

    inline bool done() const
    {
        return (iteration_ < 0) ||
               (index_[0] < min_[0]) ||
               (index_[0] > max_[0]) ||
               (index_[1] < min_[1]) ||
               (index_[1] > max_[1]);
    }

private:
    inline NDTIterator &iterateDx()
    {
        error_    += error_inc_;
        index_[0] += step_[0];
        if (error_ > 0) {
            index_[1] += step_[1];
            --error_;
        }
        --iteration_;
        return *this;
    }

    inline NDTIterator &iterateDy()
    {
        error_    += error_inc_;
        index_[1] += step_[1];
        if (error_ > 0) {
            index_[0] += step_[0];
            --error_;
        }
        --iteration_;
        return *this;
    }

    index_t min_;
    index_t max_;
    index_t index_;
    index_t step_;
    T       error_;
    T       error_inc_;
    int     iteration_;

    NDTIterator&    (NDTIterator::*iterate_)();
};
}
}

#endif // CSLIBS_MATH_2D_NDT_ITERATOR_HPP
