#ifndef CSLIBS_MATH_FRAMED_HPP
#define CSLIBS_MATH_FRAMED_HPP

#include <string>
#include <memory>

namespace cslibs_math {
namespace common {
template<typename T>
class Framed
{
public:
    using Ptr = std::shared_ptr<Framed<T>>;

    inline explicit Framed(const T &data,
                           const std::string &frame_id) :
        data_(data),
        frame_id_(frame_id)
    {
    }

    inline std::string & frameId()
    {
        return frame_id_;
    }

    inline std::string const & frameId() const
    {
        return frameId;
    }

    inline T & data()
    {
        return data_;
    }

    inline T const & data() const
    {
        return data_;
    }

    inline operator T()
    {
        return data_;
    }

    inline operator T&()
    {
        return data_;
    }

    inline operator T*()
    {
        return &data_;
    }

    inline operator const T&() const
    {
        return data_;
    }

    inline operator T () const
    {
        return data_;
    }
private:
    T              data_;
    std::string    frame_id_;
};
}
}

#endif // CSLIBS_MATH_FRAMED_HPP
