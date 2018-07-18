#ifndef CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP
#define CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP

#include <sensor_msgs/PointCloud2.h>

#include <cslibs_time/time_frame.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

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
    pcl::PointCloud<pcl::PointXYZ> tmp;
    std::vector<int> indices;
    pcl::fromROSMsg(*src, tmp);
    pcl::removeNaNFromPointCloud(tmp, tmp, indices);

    dst.reset(new cslibs_math_3d::Pointcloud3d);
    for (const pcl::PointXYZ &p : tmp)
        dst->insert(cslibs_math_3d::Point3d(p.x, p.y, p.z));
}
}
}
}

#endif // CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP
