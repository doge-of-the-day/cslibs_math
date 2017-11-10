#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <cslibs_math/linear/vector.hpp>

#include <memory>
#include <vector>

namespace cslibs_math {
namespace linear {

template<typename point_t>
class Pointcloud
{
public:
    using Ptr              = std::shared_ptr<Pointcloud>;
    using points_t         = std::vector<point_t>;
    using const_iterator_t = typename points_t::const_iterator;
    using iterator_t       = typename points_t::iterator;

    Pointcloud()
    {
    }

    Pointcloud(const std::size_t size) :
        data_(size)
    {
    }

    virtual ~Pointcloud()
    {
    }

    inline void insert(const point_t &pt)
    {
        data_.emplace_back(pt);
    }

    inline void insertInvalid()
    {
        data_.emplace_back(point_t(std::numeric_limits<double>::infinity()));
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

    inline point_t const & at(const std::size_t i) const
    {
        return data_.at(i);
    }

    inline std::vector<point_t> const & getPoints() const
    {
        return data_;
    }

    inline std::size_t size() const
    {
        return data_.size();
    }

    inline point_t min() const
    {
        point_t min(std::numeric_limits<double>::max());
        std::for_each(data_.begin(), data_.end(),
                      [&min](const point_t &p){min = min.min(p);});
        return min;
    }

    inline point_t max() const
    {
        point_t max(std::numeric_limits<double>::min());
        std::for_each(data_.begin(), data_.end(),
                      [&max](const point_t &p){max = max.max(p);});
        return max;
    }

protected:
    std::vector<point_t> data_;

};
}
}

#endif // POINTCLOUD_HPP
