#ifndef CSLIBS_MATH_3D_NDT_ITERATOR_HPP
#define CSLIBS_MATH_3D_NDT_ITERATOR_HPP

#include <memory>
#include <cslibs_math/common/array.hpp>
#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
namespace algorithms {

class EIGEN_ALIGN16 NDTIterator
{
public:
    using Ptr           = std::shared_ptr<NDTIterator>;
    using index_t       = std::array<int, 3>;
    using error_t       = std::array<int, 2>;
    using point_t       = Point3d;
    using interation_t  = NDTIterator& (NDTIterator::*)() ;
    using done_t        = bool (NDTIterator::*)();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline NDTIterator() :
        start_{{0,0,0}},
        end_  {{0,0,0}},
        index_{{0,0,0}},
        delta_abs_{{0,0,0}},
        iteration_(0)
    {
    }

    inline explicit NDTIterator(const point_t &p0,
                                const point_t &p1,
                                const double resolution):
        NDTIterator({{static_cast<int>(std::floor(p0(0) / resolution)),
                      static_cast<int>(std::floor(p0(1) / resolution)),
                      static_cast<int>(std::floor(p0(2) / resolution))}},
                    {{static_cast<int>(std::floor(p1(0) / resolution)),
                      static_cast<int>(std::floor(p1(1) / resolution)),
                      static_cast<int>(std::floor(p1(2) / resolution))}})
    {
    }

    inline explicit NDTIterator(const index_t &start,
                                const index_t &end) :
        start_(start),
        end_(end),
        index_(start_),
        error_{{0,0}},
        iteration_(0)
    {
        delta_             = end_ - start_;
        delta_abs_         = std::abs(delta_);
        delta_abs_times_2_ = delta_abs_ * 2;
        step_              = std::compare(start_, end_);

        index_last_[0] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[0])));
        index_last_[1] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[1])));
        index_last_[2] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[2])));

        end_last_[0] = static_cast<int>(std::floor(0.5 * static_cast<double>(end_[0])));
        end_last_[1] = static_cast<int>(std::floor(0.5 * static_cast<double>(end_[1])));
        end_last_[2] = static_cast<int>(std::floor(0.5 * static_cast<double>(end_[2])));

        if(delta_abs_[0] >= delta_abs_[1] && delta_abs_[0] >= delta_abs_[2]) {
            iterate_ = &NDTIterator::iterateDx;
            done_    = &NDTIterator::doneDx;
            error_[0] = delta_abs_times_2_[1] - delta_abs_[0];
            error_[1] = delta_abs_times_2_[2] - delta_abs_[0];
        } else if(delta_abs_[1] >= delta_abs_[0] && delta_abs_[1] >= delta_abs_[2]) {
            iterate_ = &NDTIterator::iterateDy;
            done_    = &NDTIterator::doneDy;
            error_[0] = delta_abs_times_2_[0] - delta_abs_[1];
            error_[1] = delta_abs_times_2_[2] - delta_abs_[1];
        } else {
            iterate_ = &NDTIterator::iterateDz;
            done_    = &NDTIterator::doneDz;
            error_[0] = delta_abs_times_2_[1] - delta_abs_[2];
            error_[1] = delta_abs_times_2_[0] - delta_abs_[2];
        }
    }

    inline virtual ~NDTIterator()
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

    inline NDTIterator& operator++()
    {
        auto do_iterate = [this]() {
            index_t li = index_last_;
            do {
                (this->*iterate_)();
            } while (!(this->*done_)() && index_last_ == li);
        };
        return (this->*done_)() ? *this : do_iterate();
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
        return (this->*done_)();
    }

private:
    inline NDTIterator &iterateDx()
    {
        if (error_[0] > 0) {
            index_[1] += step_[1];
            error_[0] -= delta_abs_times_2_[0];
            index_last_[1] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[1])));
        }
        if (error_[1] > 0) {
            index_[2] += step_[2];
            error_[1] -= delta_abs_times_2_[0];
            index_last_[2] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[2])));
        }
        error_[0] += delta_abs_times_2_[1];
        error_[1] += delta_abs_times_2_[2];
        index_[0] += step_[0];
        index_last_[0] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[0])));
        ++iteration_;
        return *this;
    }
    inline bool doneDx() const
    {
        return iteration_ >= delta_abs_[0] ||
                (index_last_[0] == end_last_[0] && index_last_[1] == end_last_[1] && index_last_[2] == end_last_[2]);
    }

    inline NDTIterator &iterateDy()
    {
        if (error_[0] > 0) {
            index_[0] += step_[0];
            error_[0] -= delta_abs_times_2_[1];
            index_last_[0] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[0])));
        }
        if (error_[1] > 0) {
            index_[2] += step_[2];
            error_[1] -= delta_abs_times_2_[1];
            index_last_[2] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[2])));
        }
        error_[0] += delta_abs_times_2_[0];
        error_[1] += delta_abs_times_2_[2];
        index_[1] += step_[1];
        index_last_[1] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[1])));
        ++iteration_;
        return *this;
    }
    inline bool doneDy() const
    {
        return iteration_ >= delta_abs_[1] ||
                (index_last_[0] == end_last_[0] && index_last_[1] == end_last_[1] && index_last_[2] == end_last_[2]);
    }

    inline NDTIterator &iterateDz()
    {
        if (error_[0] > 0) {
            index_[1] += step_[1];
            error_[0] -= delta_abs_times_2_[2];
            index_last_[1] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[1])));
        }
        if (error_[1] > 0) {
            index_[0] += step_[0];
            error_[1] -= delta_abs_times_2_[2];
            index_last_[0] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[0])));
        }
        error_[0] += delta_abs_times_2_[1];
        error_[1] += delta_abs_times_2_[0];
        index_[2] += step_[2];
        index_last_[2] = static_cast<int>(std::floor(0.5 * static_cast<double>(index_[2])));
        ++iteration_;
        return *this;
    }
    inline bool doneDz() const
    {
        return iteration_ >= delta_abs_[2] ||
                (index_last_[0] == end_last_[0] && index_last_[1] == end_last_[1] && index_last_[2] == end_last_[2]);
    }

    index_t         start_;
    index_t         end_;
    index_t         index_;
    index_t         step_;
    index_t         delta_;
    index_t         delta_abs_;
    index_t         delta_abs_times_2_;
    index_t         index_last_;
    index_t         end_last_;
    error_t         error_;

    NDTIterator&    (NDTIterator::*iterate_)() ;
    bool            (NDTIterator::*done_)() const;
    int             iteration_;
};

}
}

#endif // CSLIBS_MATH_3D_NDT_ITERATOR_HPP
