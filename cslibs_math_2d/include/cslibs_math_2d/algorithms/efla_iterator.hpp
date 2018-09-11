#ifndef CSLIBS_MATH_2D_EFLA_ITERATOR_HPP
#define CSLIBS_MATH_2D_EFLA_ITERATOR_HPP

#include <memory>
#include <cslibs_math/common/array.hpp>
#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_math_2d {
namespace algorithms {
class EIGEN_ALIGN16 EFLAIterator
{
public:
    using Ptr           = std::shared_ptr<EFLAIterator>;
    using index_t       = std::array<int, 2>;
    using point_t       = Point2d;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline explicit EFLAIterator(const index_t &start,
                                 const index_t &end) :
        start_(start),
        end_(end),
        index_(start_),
        j_(0.0)
    {
        int short_len = end_[1] - start_[1];
        int long_len  = end_[0] - start_[0];

        bool y_longer = false;
        if (std::fabs(short_len) > std::fabs(long_len)) {
            std::swap(short_len, long_len);
            y_longer = true;
        }

        increment_val_ = std::copysign(1.0, long_len);
        dec_inc_ = (long_len == 0) ?
                    static_cast<double>(short_len) :
                    (static_cast<double>(short_len) / static_cast<double>(std::fabs(long_len)));

        iterate_ = y_longer ? &EFLAIterator::iterateY : &EFLAIterator::iterateX;
    }

    inline explicit EFLAIterator(const point_t &p0,
                                 const point_t &p1,
                                 const double  &resolution) :
        EFLAIterator({{static_cast<int>(std::floor(p0(0) / resolution)),
                       static_cast<int>(std::floor(p0(1) / resolution))}},
                     {{static_cast<int>(std::floor(p1(0) / resolution)),
                       static_cast<int>(std::floor(p1(1) / resolution))}})
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

    inline EFLAIterator& operator++()
    {
        return done() ? *this : (this->*iterate_)();
    }

    inline int length2() const
    {
        auto sq = [](const int d) { return d*d;};
        return sq(index_[0] - end_[0]) +
               sq(index_[1] - end_[1]);
    }

    inline bool done() const
    {
        return index_[0] == end_[0] &&
               index_[1] == end_[1];
    }

private:
    index_t         start_;
    index_t         end_;
    index_t         index_;
    int             increment_val_;
    double          j_;
    double          dec_inc_;

    EFLAIterator&   (EFLAIterator::*iterate_)();

    inline EFLAIterator &iterateX()
    {
        j_ += dec_inc_;

        index_[0] += increment_val_;
        index_[1]  = start_[1] + static_cast<int>(std::round(j_));

        return *this;
    }

    inline EFLAIterator &iterateY()
    {
        j_ += dec_inc_;

        index_[0]  = start_[0] + static_cast<int>(std::round(j_));
        index_[1] += increment_val_;

        return *this;
    }
};
}
}

#endif // CSLIBS_MATH_2D_EFLA_ITERATOR_HPP
