#ifndef CSLIBS_MATH_ROS_TF_LISTENER_3D_HPP
#define CSLIBS_MATH_ROS_TF_LISTENER_3D_HPP

#include <cslibs_utility/common/delegate.hpp>
#include <cslibs_time/stamped.hpp>

#include <cslibs_math_ros/tf/conversion_3d.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <memory>
#include <mutex>


namespace cslibs_math_ros {
namespace tf {
class TFListener3d {
public:
    using Ptr       = std::shared_ptr<TFListener3d>;
    using stamped_t = cslibs_time::Stamped<cslibs_math_3d::Transform3d>;
    using mutex_t   = std::mutex;
    using lock_t    = std::unique_lock<mutex_t>;

    inline TFListener3d()   = default;
    virtual ~TFListener3d() = default;

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                stamped_t            &transform)
    {
        ::tf::Transform tf_transform;
        if(lookupTransform(target_frame, source_frame,time, tf_transform)) {
            transform.data() = from(tf_transform);
            transform.stamp() = cslibs_time::Time(time.toNSec());
            return true;
        }
        return false;
    }

    inline bool lookupTransform(const std::string           &target_frame,
                                const std::string           &source_frame,
                                const ros::Time             &time,
                                stamped_t                   &transform,
                                const ros::Duration         &timeout)
    {
        ::tf::Transform tf_transform;
        if(lookupTransform(target_frame,
                           source_frame,
                           time,
                           tf_transform,
                           timeout)) {
            transform.data()  = from(tf_transform);
            transform.stamp() = cslibs_time::Time(time.toNSec());
            return true;
        }
        return false;
    }


    inline bool lookupTransform(const std::string              &target_frame,
                                const std::string              &source_frame,
                                const ros::Time                &time,
                                cslibs_math_3d::Transform3d    &transform)
    {
        ::tf::Transform tf_transform;
        if(lookupTransform(target_frame, source_frame, time, tf_transform)) {
            transform = from(tf_transform);
            return true;
        }
        return false;
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                cslibs_math_3d::Transform3d    &transform,
                                const ros::Duration  &timeout)
    {
        ::tf::Transform tf_transform;
        if(lookupTransform(target_frame,
                           source_frame,
                           time,
                           tf_transform,
                           timeout)) {
            transform = from(tf_transform);
            return true;
        }
        return false;
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                ::tf::StampedTransform &transform)
    {
        lock_t l(mutex_);
        std::string error;
        if(tf_.canTransform(target_frame, source_frame, time, &error)) {
            tf_.lookupTransform(target_frame, source_frame, time, transform);
            return true;
        } else {
            return false;
        }
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                ::tf::StampedTransform &transform,
                                const ros::Duration  &timeout)
    {
        lock_t l(mutex_);
        if(tf_.waitForTransform(target_frame, source_frame, time, timeout)) {
            tf_.lookupTransform(target_frame, source_frame, time, transform);
            return true;
        }
        return false;
    }


    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                ::tf::Transform        &transform)
    {
        lock_t l(mutex_);
        std::string error;
        if(tf_.canTransform(target_frame, source_frame, time, &error)) {
            ::tf::StampedTransform stamped;
            tf_.lookupTransform(target_frame, source_frame, time, stamped);
            transform = static_cast<::tf::Transform>(stamped);
            return true;
        } else {
            return false;
        }
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                ::tf::Transform        &transform,
                                const ros::Duration  &timeout)
    {
        lock_t l(mutex_);
        ::tf::StampedTransform stamped;
        if(tf_.waitForTransform(target_frame, source_frame, time, timeout)) {
            tf_.lookupTransform(target_frame, source_frame, time, stamped);
            transform = static_cast<::tf::Transform>(stamped);
            return true;
        }
        return false;
    }

    inline bool canTransform(const std::string &target_frame,
                             const std::string &source_frame,
                             const ros::Time   &time)
    {
        lock_t l(mutex_);
        return tf_.canTransform(target_frame, source_frame, time);
    }

    inline bool waitForTransform(const std::string &target_frame,
                                 const std::string &source_frame,
                                 const ros::Time &time,
                                 const ros::Duration &timeout)
    {
        lock_t l(mutex_);
        return tf_.waitForTransform(target_frame, source_frame, time, timeout);
    }

    inline void getFrameStrings(std::vector<std::string> &frames)
    {
        lock_t l(mutex_);
        tf_.getFrameStrings(frames);
    }

protected:
    std::mutex              mutex_;
    ::tf::TransformListener tf_;
};
}
}
#endif // CSLIBS_MATH_ROS_TF_LISTENER_3D_HPP
