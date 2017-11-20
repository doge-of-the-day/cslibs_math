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

    Pointcloud() :
        min_(std::numeric_limits<typename point_t::type_t>::max()),
        max_(std::numeric_limits<typename point_t::type_t>::min())
    {
    }

    virtual ~Pointcloud()
    {
    }

    inline virtual void insert(const point_t &pt)
    {
        min_.min(pt);
        max_.max(pt);
        data_.emplace_back(pt);
    }

    inline virtual void insertInvalid()
    {
        data_.emplace_back(point_t(std::numeric_limits<double>::infinity()));
    }

    inline virtual void clear()
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
        return min_;
    }

    inline point_t max() const
    {
        return max_;
    }

    template<typename transform_t>
    inline void transform(const transform_t &t)
    {
        auto apply = [&t, this](point_t &p){
             p = t * p;
             min_.min(p);
             max_.max(p);
        };

        min_ = point_t(std::numeric_limits<typename point_t::type_t>::max());
        max_ = point_t(std::numeric_limits<typename point_t::type_t>::min());
        std::for_each(data_.begin(), data_.end(),
                      apply);
    }


protected:
    std::vector<point_t> data_;
    point_t              min_;
    point_t              max_;
};
}
}

#endif // POINTCLOUD_HPP
