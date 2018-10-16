#ifndef CSLIBS_MATH_ROS_TF_LISTENER_2D_HPP
#define CSLIBS_MATH_ROS_TF_LISTENER_2D_HPP

#include <cslibs_time/stamped.hpp>

#include <cslibs_math_ros/tf/conversion_2d.hpp>

#include <cslibs_math/utility/stamped.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <memory>
#include <mutex>


namespace cslibs_math_ros {
namespace tf {
class TFListener2d {

#warning TFListener2d is deprecated.

public:
    using Ptr       = std::shared_ptr<TFListener2d>;
    using stamped_t = cslibs_math::utility::Stamped<cslibs_math_2d::Transform2d>;
    using mutex_t   = std::mutex;
    using lock_t    = std::unique_lock<mutex_t>;

    inline TFListener2d()   = default;
    virtual ~TFListener2d() = default;

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                stamped_t            &transform)
    {
        assert (target_frame != "");
        assert (source_frame != "");

        ::tf::Transform tf_transform;
        if(lookupTransform(target_frame, source_frame,time, tf_transform)) {
            transform.data() =  conversion_2d::from(tf_transform);
            transform.stamp() = cslibs_time::Time(time.toNSec()).time();
            return true;
        }
        return false;
    }

    inline bool lookupTransform(const std::string           &target_frame,
                                const std::string           &source_frame,
                                const ros::Time             &time,
                                stamped_t    &transform,
                                const ros::Duration         &timeout)
    {
        assert (target_frame != "");
        assert (source_frame != "");

        ::tf::Transform tf_transform;
        if(lookupTransform(target_frame,
                           source_frame,
                           time,
                           tf_transform,
                           timeout)) {
            transform.data()  = conversion_2d::from(tf_transform);
            transform.stamp() = cslibs_time::Time(time.toNSec()).time();
            return true;
        }
        return false;
    }


    inline bool lookupTransform(const std::string              &target_frame,
                                const std::string              &source_frame,
                                const ros::Time                &time,
                                cslibs_math_2d::Transform2d    &transform)
    {
        assert (target_frame != "");
        assert (source_frame != "");

        ::tf::Transform tf_transform;
        if(lookupTransform(target_frame, source_frame, time, tf_transform)) {
            transform = conversion_2d::from(tf_transform);
            return true;
        }
        return false;
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                cslibs_math_2d::Transform2d    &transform,
                                const ros::Duration  &timeout)
    {
        assert (target_frame != "");
        assert (source_frame != "");

        ::tf::Transform tf_transform;
        if(lookupTransform(target_frame,
                           source_frame,
                           time,
                           tf_transform,
                           timeout)) {
            transform = conversion_2d::from(tf_transform);
            return true;
        }
        return false;
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                ::tf::StampedTransform &transform)
    {
        assert (target_frame != "");
        assert (source_frame != "");

        lock_t l(mutex_);
        std::string error;
        if(tf_.canTransform(target_frame, source_frame, time, &error)) {
            tf_.lookupTransform(target_frame, source_frame, time, transform);
            return true;
        } else {
            return false;
        }
    }

    inline bool lookupTransform(const std::string      &target_frame,
                                const std::string      &source_frame,
                                const ros::Time        &time,
                                ::tf::StampedTransform &transform,
                                const ros::Duration    &timeout)
    {
        assert (target_frame != "");
        assert (source_frame != "");

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
        assert (target_frame != "");
        assert (source_frame != "");

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
                                ::tf::Transform      &transform,
                                const ros::Duration  &timeout)
    {
        assert (target_frame != "");
        assert (source_frame != "");

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
        assert (target_frame != "");
        assert (source_frame != "");

        lock_t l(mutex_);
        return tf_.canTransform(target_frame, source_frame, time);
    }

    inline bool waitForTransform(const std::string &target_frame,
                                 const std::string &source_frame,
                                 const ros::Time &time,
                                 const ros::Duration &timeout)
    {
        assert (target_frame != "");
        assert (source_frame != "");

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
#endif // CSLIBS_MATH_ROS_TF_LISTENER_2D_HPP
