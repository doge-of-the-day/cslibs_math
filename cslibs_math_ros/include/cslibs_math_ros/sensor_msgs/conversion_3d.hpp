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

inline void from(const cslibs_math_3d::Pointcloud3d::Ptr &src,
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

inline void from(const ::sensor_msgs::PointCloud2ConstPtr &src,
                 cslibs_math_3d::PointcloudRGB3d::Ptr &dst)
{
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_x(*src, "x");
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_y(*src, "y");
    ::sensor_msgs::PointCloud2ConstIterator<float> iter_z(*src, "z");
    ::sensor_msgs::PointCloud2ConstIterator<u_int8_t> iter_r(*src, "r");
    ::sensor_msgs::PointCloud2ConstIterator<u_int8_t> iter_g(*src, "g");
    ::sensor_msgs::PointCloud2ConstIterator<u_int8_t> iter_b(*src, "b");
    ::sensor_msgs::PointCloud2ConstIterator<u_int8_t> iter_a(*src, "a");


    dst.reset(new cslibs_math_3d::PointcloudRGB3d);
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b, ++iter_a) {
        if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
            cslibs_math_3d::Point3d p(*iter_x, *iter_y, *iter_z);
            cslibs_math::color::Color c(static_cast<float>(*iter_r)/256.0f,
                                        static_cast<float>(*iter_g)/256.0f,
                                        static_cast<float>(*iter_b)/256.0f);
            cslibs_math_3d::PointRGB3d point(p, static_cast<float>(*iter_a)/256.0, c);
            if (p.isNormal())
                dst->insert(point);
        }
    }
}

inline void from(const cslibs_math_3d::PointcloudRGB3d::Ptr &src,
                 ::sensor_msgs::PointCloud2 &dst)
{
    // metadata
    dst.width        = src->size();
    dst.height       = 1;
    dst.is_dense     = false;
    dst.is_bigendian = false;


    ::sensor_msgs::PointCloud2Modifier modifier(dst);
    modifier.setPointCloud2FieldsByString(2,"xyz","rgba");
    modifier.resize(src->size());

    ::sensor_msgs::PointCloud2Iterator<float> iter_x(dst, "x");
    ::sensor_msgs::PointCloud2Iterator<float> iter_y(dst, "y");
    ::sensor_msgs::PointCloud2Iterator<float> iter_z(dst, "z");
    ::sensor_msgs::PointCloud2Iterator<u_int8_t> iter_r(dst, "r");
    ::sensor_msgs::PointCloud2Iterator<u_int8_t> iter_g(dst, "g");
    ::sensor_msgs::PointCloud2Iterator<u_int8_t> iter_b(dst, "b");
    ::sensor_msgs::PointCloud2Iterator<u_int8_t> iter_a(dst, "a");

    for (const auto &p : *src) {
        cslibs_math_3d::Point3d pos = p.getPoint();
        cslibs_math::color::Color c = p.getColor();
        float a = p.getAlpha();
        *iter_x = static_cast<float>(pos(0));
        *iter_y = static_cast<float>(pos(1));
        *iter_z = static_cast<float>(pos(2));
        *iter_r = static_cast<u_int8_t>(c.r*255.0f);
        *iter_g = static_cast<u_int8_t>(c.g*255.0f);
        *iter_b = static_cast<u_int8_t>(c.b*255.0f);
        *iter_a = static_cast<u_int8_t>(a*255.0f);
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
        ++iter_a;
    }
}

}
}
}

#endif // CSLIBS_MATH_ROS_SENSOR_MSGS_CONVERSION_3D_HPP
