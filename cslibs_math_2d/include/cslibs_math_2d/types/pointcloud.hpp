#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <cslibs_math_2d/types/point.hpp>
#include <cslibs_math_2d/types/transform.hpp>

namespace cslibs_math_2d {
class Pointcloud2d
{
public:
    using Ptr              = std::shared_ptr<Pointcloud2d>;
    using points_t         = std::vector<Point2d>;
    using const_iterator_t = points_t::const_iterator;
    using iterator_t       = points_t::iterator;

    Pointcloud2d()
    {
    }

    Pointcloud2d(const std::size_t size) :
        data_(size)
    {
    }

    virtual ~Pointcloud2d()
    {
    }

    inline void insert(const Point2d &pt)
    {
        data_.emplace_back(pt);
    }

    inline void insertInvalid()
    {
        data_.emplace_back(Point2d(std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::infinity()));
    }

    inline void clear()
    {
        data_.clear();
    }

    inline const_iterator_t begin() const
    {
        return data_.begin();
    }

    inline const_iterator_t end() const
    {
        return data_.end();
    }


    inline iterator_t begin()
    {
        return data_.begin();
    }

    inline iterator_t end()
    {
        return data_.end();
    }


    inline std::vector<Point2d> const & getPoints() const
    {
        return data_;
    }

    inline Point2d min() const
    {
        Point2d min(std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max());
        std::for_each(data_.begin(), data_.end(),
                      [&min](const Point2d &p){min = min.min(p);});
    }

    inline Point2d max() const
    {
        Point2d min(std::numeric_limits<double>::min(),
                    std::numeric_limits<double>::min());
        std::for_each(data_.begin(), data_.end(),
                      [&min](const Point2d &p){min = min.max(p);});
    }

    virtual inline void transform(const Transform2d &t)
    {
        std::for_each(data_.begin(), data_.end(),
                      [&t](Point2d &p){p = t * p;});
    }

protected:
    std::vector<Point2d>        data_;

};
}



#endif // POINTCLOUD_HPP
