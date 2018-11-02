#pragma once

#include <cslibs_time/stamped.hpp>
#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math_3d/linear/transform.hpp>

#include <ros/time.h>
#include <ros/duration.h>
#include <memory>

namespace cslibs_math_ros {
namespace tf {

class TFProvider
{
public:
    using Ptr = std::shared_ptr<TFProvider>;
    using stamped_2d_t = cslibs_time::Stamped<cslibs_math_2d::Transform2d>;
    using stamped_3d_t = cslibs_time::Stamped<cslibs_math_3d::Transform3d>;

    virtual ~TFProvider() = default;

    // 2d interface
    virtual bool lookupTransform(const std::string& target_frame,
                                 const std::string& source_frame,
                                 const ros::Time& time,
                                 stamped_2d_t& transform,
                                 const ros::Duration& timeout) = 0;

    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         stamped_2d_t& transform)
    {
        return lookupTransform(target_frame, source_frame, time, transform, {});
    }

    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         cslibs_math_2d::Transform2d& transform,
                         const ros::Duration& timeout = {})
    {
        stamped_2d_t stamped;
        if (!lookupTransform(target_frame, source_frame, time, stamped, timeout))
            return false;

        transform = stamped.data();
        return true;
    }

    // 3d interface
    virtual bool lookupTransform(const std::string& target_frame,
                                 const std::string& source_frame,
                                 const ros::Time& time,
                                 stamped_3d_t& transform,
                                 const ros::Duration& timeout) = 0;

    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         stamped_3d_t& transform)
    {
        return lookupTransform(target_frame, source_frame, time, transform, {});
    }

    bool lookupTransform(const std::string& target_frame,
                         const std::string& source_frame,
                         const ros::Time& time,
                         cslibs_math_3d::Transform3d& transform,
                         const ros::Duration& timeout = {})
    {
        stamped_3d_t stamped;
        if (!lookupTransform(target_frame, source_frame, time, stamped, timeout))
            return false;

        transform = stamped.data();
        return true;
    }

    // misc
    virtual bool canTransform(const std::string& target_frame,
                              const std::string& source_frame,
                              const ros::Time& time) = 0;

    virtual bool waitForTransform(const std::string& target_frame,
                                  const std::string& source_frame,
                                  const ros::Time& time,
                                  const ros::Duration& timeout) = 0;
};

}
}
