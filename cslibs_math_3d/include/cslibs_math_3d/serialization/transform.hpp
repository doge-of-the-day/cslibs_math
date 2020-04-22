#ifndef CSLIBS_MATH_3D_SERIALIZATION_TRANSFORM_HPP
#define CSLIBS_MATH_3D_SERIALIZATION_TRANSFORM_HPP

#include <cslibs_math/serialization/vector.hpp>
#include <cslibs_math_3d/serialization/quaternion.hpp>
#include <cslibs_math_3d/linear/quaternion.hpp>
#include <cslibs_math_3d/linear/transform.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>

#include <cslibs_math/serialization/binary.hpp>

namespace cslibs_math {
namespace serialization {
namespace transform {
namespace binary {

template <typename T>
inline static std::size_t read(std::ifstream  &in,
                               cslibs_math_3d::Transform3<T> &transform)
{
    using transform_t = cslibs_math_3d::Transform3<T>;

    T x     = io<T>::read(in);
    T y     = io<T>::read(in);
    T z     = io<T>::read(in);
    T roll  = io<T>::read(in);
    T pitch = io<T>::read(in);
    T yaw   = io<T>::read(in);

    transform = transform_t(typename transform_t::translation_t(x,y,z),
                            typename transform_t::rotation_t(roll,pitch,yaw));
    return 6ul * sizeof(T);
}

template <typename T>
inline static void  write(const cslibs_math_3d::Transform3<T> &transform,
                          std::ofstream &out)
{
    io<T>::write(transform.tx(), out);
    io<T>::write(transform.ty(), out);
    io<T>::write(transform.tz(), out);
    io<T>::write(transform.roll(), out);
    io<T>::write(transform.pitch(), out);
    io<T>::write(transform.yaw(), out);
}

}
}
}
}

namespace YAML {
template<typename T>
struct convert<cslibs_math_3d::Transform3<T>>
{
    static Node encode(const cslibs_math_3d::Transform3<T> &rhs)
    {
        Node n;
        n["t"] = rhs.translation();
        n["r"] = rhs.rotation();
        return n;
    }
    static bool decode(const Node& n, cslibs_math_3d::Transform3<T> &rhs)
    {
        if(!n.IsMap())
            return false;
        rhs.translation() = n["t"].as<cslibs_math_3d::Vector3<T>>();
        rhs.rotation()    = n["r"].as<cslibs_math_3d::Quaternion<T>>();
        return true;
    }
};
}

#endif // CSLIBS_MATH_3D_SERIALIZATION_TRANSFORM_HPP
