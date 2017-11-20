#ifndef CSLIBS_MATH_2D_ROS_HPP
#define CSLIBS_MATH_2D_ROS_HPP

#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_math_2d/linear/polar_pointcloud.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

namespace cslibs_math_2d {
namespace conversion {

inline void from(const sensor_msgs::PointCloud &src,
                 Pointcloud2d &dst,
                 const bool filter = false)
{
    if(filter) {
        for(const auto &p : src.points) {
            if(std::isnormal(p.x) && std::isnormal(p.y))
                dst.insert(Point2d(p.x, p.y));
        }
    } else {
        for(const auto &p : src.points) {
            dst.insert(Point2d(p.x, p.y));
        }

    }
}

inline void from(const sensor_msgs::LaserScan &src,
                 Pointcloud2d &dst,
                 const bool filter = false)
{
    float angle = src.angle_min;
    if(filter) {
        for(const float r : src.ranges) {
            if(r >= src.range_min && r <= src.range_max) {
                dst.insert(Point2d(std::cos(angle) * r,
                                   std::sin(angle) * r));
            }
            angle += src.angle_increment;
        }
    } else {
        for(const float r : src.ranges) {
            if(r >= src.range_min && r <= src.range_max) {
                dst.insert(Point2d(std::cos(angle) * r,
                                   std::sin(angle) * r));
            } else {
                dst.insertInvalid();
            }
            angle += src.angle_increment;
        }
    }
}

inline void from(const sensor_msgs::LaserScan &src,
                 PolarPointlcoud2d &dst,
                 const bool filter = false)
{
    float angle = src.angle_min;
    if(filter) {
        for(const float r : src.ranges) {
            if(r >= src.range_min && r <= src.range_max) {
                dst.insert(Point2d(angle, r));
            }
            angle += src.angle_increment;
        }
    } else {
        for(const float r : src.ranges) {
            if(r >= src.range_min && r <= src.range_max) {
                dst.insert(PolarPoint2d(static_cast<double>(angle),
                                        static_cast<double>(r)));;
            } else {
                dst.insert(PolarPoint2d());
            }
            angle += src.angle_increment;
        }
    }
}

inline void from(const Pointcloud2d &src,
                 sensor_msgs::PointCloud &dst)
{
    for(const auto &p : src) {
        geometry_msgs::Point32 gp;
        gp.x = static_cast<float>(p(0));
        gp.y = static_cast<float>(p(1));
        dst.points.emplace_back(gp);
    }
}

inline void from(const PolarPointlcoud2d &src,
                 sensor_msgs::PointCloud &dst)
{
    for(const auto &p : src) {
        geometry_msgs::Point32 gp;
        gp.x = static_cast<float>(p.getCartesian()(0));
        gp.y = static_cast<float>(p.getCartesian()(1));
        dst.points.emplace_back(gp);
    }
}
}
}



#endif // CSLIBS_MATH_2D_ROS_HPP
