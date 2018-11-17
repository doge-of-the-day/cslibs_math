#ifndef CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP
#define CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cslibs_time/time_frame.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>

namespace cslibs_math_ros {
namespace sensor_msgs {
namespace conversion_3d {
inline cslibs_time::TimeFrame from(const ::sensor_msgs::PointCloud2ConstPtr &src)
{
    const ros::Time start_stamp = src->header.stamp;
    return cslibs_time::TimeFrame(start_stamp.toNSec(),
                                  start_stamp.toNSec());
}

inline void from(const ::sensor_msgs::PointCloud2ConstPtr &src,
                 cslibs_math_3d::Pointcloud3d::Ptr &dst)
{
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_x(*src, "x");
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_y(*src, "y");
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_z(*src, "z");

    dst.reset(new cslibs_math_3d::Pointcloud3d);
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
            cslibs_math_3d::Point3d p(*iter_x, *iter_y, *iter_z);
            if (p.isNormal())
                dst->insert(p);
        }
    }
}

inline void from(cslibs_math_3d::Pointcloud3d::Ptr &src,
                 ::sensor_msgs::PointCloud2 &dst)
{
    // metadata
    dst.width        = src->size();
    dst.height       = 1;
    dst.is_dense     = false;
    dst.is_bigendian = false;
    dst.point_step   = 3 * sizeof(float);
    dst.row_step     = static_cast<uint32_t>(dst.point_step * src->size());

    // fields x y z intensity
    dst.fields.resize(3);
    dst.fields[0].name = "x";
    dst.fields[1].name = "y";
    dst.fields[2].name = "z";

    std::vector<float> tmp;
    for(const auto &p : *src) {
      for(std::size_t i = 0 ; i < 3 ; ++i)
        tmp.emplace_back(static_cast<float>(p(i)));
    }
    // data
    memcpy(&dst.data[0], &tmp[0], dst.row_step);
}
}
}
}

#endif // CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP
