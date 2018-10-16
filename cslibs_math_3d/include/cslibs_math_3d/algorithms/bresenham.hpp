#ifndef CSLIBS_MATH_3D_BRESENHAM_HPP
#define CSLIBS_MATH_3D_BRESENHAM_HPP

#include <memory>
#include <cslibs_math/common/array.hpp>
#include <cslibs_math_3d/linear/point.hpp>

namespace cslibs_math_3d {
namespace algorithms {

class EIGEN_ALIGN16 Bresenham
{
public:
    using Ptr           = std::shared_ptr<Bresenham>;
    using index_t       = std::array<int, 3>;
    using error_t       = std::array<int, 2>;
    using point_t       = Point3d;
    using interation_t  = Bresenham& (Bresenham::*)() ;
    using done_t        = bool (Bresenham::*)();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline Bresenham() :
        start_{{0,0,0}},
        end_  {{0,0,0}},
        index_{{0,0,0}},
        delta_abs_{{0,0,0}},
        iteration_(0)
    {
    }

    inline explicit Bresenham(const point_t &p0,
                              const point_t &p1,
                              const double resolution):
        Bresenham({{static_cast<int>(p0(0) / resolution), static_cast<int>(p0(1) / resolution), static_cast<int>(p0(2) / resolution)}},
                  {{static_cast<int>(p1(0) / resolution), static_cast<int>(p1(1) / resolution), static_cast<int>(p1(2) / resolution)}})
    {
    }

    inline explicit Bresenham(const index_t     &start,
                              const index_t     &end) :
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

        if(delta_abs_[0] >= delta_abs_[1] && delta_abs_[0] >= delta_abs_[2]) {
            iterate_ = &Bresenham::iterateDx;
            done_    = &Bresenham::doneDx;
            error_[0] = delta_abs_times_2_[1] - delta_abs_[0];
            error_[1] = delta_abs_times_2_[2] - delta_abs_[0];
        } else if(delta_abs_[1] >= delta_abs_[0] && delta_abs_[1] >= delta_abs_[2]) {
            iterate_ = &Bresenham::iterateDy;
            done_    = &Bresenham::doneDy;
            error_[0] = delta_abs_times_2_[0] - delta_abs_[1];
            error_[1] = delta_abs_times_2_[2] - delta_abs_[1];
        } else {
            iterate_ = &Bresenham::iterateDz;
            done_    = &Bresenham::doneDz;
            error_[0] = delta_abs_times_2_[1] - delta_abs_[2];
            error_[1] = delta_abs_times_2_[0] - delta_abs_[2];
        }
    }

    inline virtual ~Bresenham()
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


    inline Bresenham& operator++()
    {
        return (this->*done_)() ? *this : (this->*iterate_)();
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
    inline Bresenham &iterateDx()
    {
        if (error_[0] > 0) {
            index_[1] += step_[1];
            error_[0] -= delta_abs_times_2_[0];
        }
        if (error_[1] > 0) {
            index_[2] += step_[2];
            error_[1] -= delta_abs_times_2_[0];
        }
        error_[0] += delta_abs_times_2_[1];
        error_[1] += delta_abs_times_2_[2];
        index_[0] += step_[0];
        ++iteration_;
        return *this;
    }
    inline bool doneDx() const
    {
        return iteration_ >= delta_abs_[0];
    }

    inline Bresenham &iterateDy()
    {
        if (error_[0] > 0) {
            index_[0] += step_[0];
            error_[0] -= delta_abs_times_2_[1];
        }
        if (error_[1] > 0) {
            index_[2] += step_[2];
            error_[1] -= delta_abs_times_2_[1];
        }
        error_[0] += delta_abs_times_2_[0];
        error_[1] += delta_abs_times_2_[2];
        index_[1] += step_[1];
        ++iteration_;
        return *this;
    }
    inline bool doneDy() const
    {
        return iteration_ >= delta_abs_[1];
    }

    inline Bresenham &iterateDz()
    {
        if (error_[0] > 0) {
            index_[1] += step_[1];
            error_[0] -= delta_abs_times_2_[2];
        }
        if (error_[1] > 0) {
            index_[0] += step_[0];
            error_[1] -= delta_abs_times_2_[2];
        }
        error_[0] += delta_abs_times_2_[1];
        error_[1] += delta_abs_times_2_[0];
        index_[2] += step_[2];
        ++iteration_;
        return *this;
    }
    inline bool doneDz() const
    {
        return iteration_ >= delta_abs_[2];
    }

    index_t         start_;
    index_t         end_;
    index_t         index_;
    index_t         step_;
    index_t         delta_;
    index_t         delta_abs_;
    index_t         delta_abs_times_2_;
    error_t         error_;
    Bresenham&     (Bresenham::*iterate_)() ;
    bool           (Bresenham::*done_)() const;
    int             iteration_;

};

}
}

#endif // CSLIBS_MATH_3D_BRESENHAM_HPP
