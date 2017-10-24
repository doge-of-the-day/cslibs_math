#ifndef POINTCLOUD_WITH_ORIGIN_HPP
#define POINTCLOUD_WITH_ORIGIN_HPP

#include <cslibs_math_2d/types/pointcloud.hpp>

namespace cslibs_math_2d {
class Pointcloud2dWithOrigin : public Pointcloud2d
{
public:
    Pointcloud2dWithOrigin(const Transform2d &origin = Transform2d::identity()) :
        Pointcloud2d(),
        origin_(origin)
    {
    }
    Pointcloud2dWithOrigin(const std::size_t size,
                           const Transform2d &origin = Transform2d::identity()) :
        Pointcloud2d(size),
        origin_(origin)
    {
    }

    virtual inline void transform(const Transform2d &t) override
    {
        origin_ *= t;
    }

    virtual inline void globalize()
    {
        transform(origin_);
        origin_ = Transform2d::identity();
    }

protected:
    Transform2d origin_;

};
}


#endif // POINTCLOUD_WITH_ORIGIN_HPP
