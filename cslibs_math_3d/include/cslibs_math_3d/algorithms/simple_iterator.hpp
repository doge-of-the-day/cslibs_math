#ifndef CSLIBS_MATH_3D_SIMPLE_ITERATOR_HPP
#define CSLIBS_MATH_3D_SIMPLE_ITERATOR_HPP

#include <memory>
#include <cslibs_math/common/array.hpp>
#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
namespace algorithms {
class EIGEN_ALIGN16 SimpleIterator
{
public:
    using Ptr           = std::shared_ptr<SimpleIterator>;
    using index_t       = std::array<int, 3>;
    using point_t       = Point3d;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline explicit SimpleIterator(const index_t &start,
                                   const index_t &end) :
        SimpleIterator(point_t(start[0], start[1], start[2]), point_t(end[0], end[1], end[2]), 1.0)
    {
    }

    inline explicit SimpleIterator(const point_t &p0,
                                   const point_t &p1,
                                   const double  &resolution) :
        start_({{static_cast<int>(std::floor(p0(0) / resolution)),
                 static_cast<int>(std::floor(p0(1) / resolution)),
                 static_cast<int>(std::floor(p0(2) / resolution))}}),
        end_({{static_cast<int>(std::floor(p1(0) / resolution)),
               static_cast<int>(std::floor(p1(1) / resolution)),
               static_cast<int>(std::floor(p1(2) / resolution))}}),
        resolution_(resolution),
        diff_((p1-p0) * resolution),
        point_(0.0, 0.0, 0.0),
        index_(start_),
        min_(std::min(start_, end_)),
        max_(std::max(start_, end_))
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

    inline SimpleIterator& operator++()
    {
        if (done())
            return *this;

        index_t i({{index_[0], index_[1], index_[2]}});
        do {
            point_ += diff_;
            i[0] = std::max(min_[0], std::min(max_[0], static_cast<int>(std::round(start_[0] + point_(0)/resolution_))));
            i[1] = std::max(min_[1], std::min(max_[1], static_cast<int>(std::round(start_[1] + point_(1)/resolution_))));
            i[2] = std::max(min_[2], std::min(max_[2], static_cast<int>(std::round(start_[2] + point_(2)/resolution_))));
        } while (i[0] == index_[0] && i[1] == index_[1] && i[2] == index_[2]);
        index_ = i;

        return *this;
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
    index_t         start_;
    index_t         end_;
    double          resolution_;
    point_t         diff_;
    point_t         point_;
    index_t         index_;
    index_t         min_;
    index_t         max_;
};
}
}

#endif // CSLIBS_MATH_3D_SIMPLE_ITERATOR_HPP
