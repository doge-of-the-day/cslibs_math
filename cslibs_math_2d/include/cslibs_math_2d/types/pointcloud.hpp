#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <cslibs_math_2d/types/point.hpp>
#include <cslibs_math_2d/types/transform.hpp>

namespace cslibs_math_2d {
class Pointcloud2d
{
public:
    struct Point2d {
        const cslibs_math_2d::Point2d point;
        const bool                    valid;

        inline Point2d() :
            point(std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity()),
            valid(false)
        {
        }

        inline Point2d(const cslibs_math_2d::Point2d &point,
                       const bool valid = true) :
                point(point),
                valid(valid)
        {
        }

        inline Point2d(const Point2d &other) :
            point(other.point),
            valid(other.valid)
        {
        }

        inline Point2d(Point2d &&other) :
            point(std::move(other.point)),
            valid(other.valid)
        {
        }
    } __attribute__ ((aligned (32)));

    using Ptr              = std::shared_ptr<Pointcloud2d>;
    using points_t         = std::vector<Point2d>;
    using const_iterator_t = points_t::const_iterator;

    Pointcloud2d() :
        min_(std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()),
        max_(std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest())
    {
    }

    Pointcloud2d(const std::size_t size) :
        points_(size),
        min_(std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()),
        max_(std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest())
    {
    }

    virtual ~Pointcloud2d()
    {
    }

    inline void insert(const cslibs_math_2d::Point2d &pt)
    {
        min_ = min_.min(pt);
        max_ = max_.max(pt);
        points_.emplace_back(pt);
    }

    inline void insertInvalid()
    {
        points_.emplace_back(Point2d());
    }

    inline void clear()
    {
        min_.x() = std::numeric_limits<double>::max();
        min_.y() = std::numeric_limits<double>::max();
        max_.x() = std::numeric_limits<double>::lowest();
        max_.y() = std::numeric_limits<double>::lowest();
        points_.clear();
    }

    inline const_iterator_t begin() const
    {
        return points_.begin();
    }

    inline const_iterator_t end() const
    {
        return points_.end();
    }

    inline std::vector<Point2d> const & getPoints() const
    {
        return points_;
    }

protected:
    std::vector<Point2d>        points_;
    cslibs_math_2d::Point2d     min_;
    cslibs_math_2d::Point2d     max_;

};
}

#endif // POINTCLOUD_HPP
