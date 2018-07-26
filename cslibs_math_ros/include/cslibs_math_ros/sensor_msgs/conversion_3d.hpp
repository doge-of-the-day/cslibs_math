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
}
}
}

#endif // CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP
