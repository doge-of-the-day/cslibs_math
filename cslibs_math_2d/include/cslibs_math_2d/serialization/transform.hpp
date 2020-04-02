#ifndef CSLIBS_MATH_2D_SERIALIZATION_TRANSFORM_HPP
#define CSLIBS_MATH_2D_SERIALIZATION_TRANSFORM_HPP

#include <cslibs_math/serialization/vector.hpp>
#include <cslibs_math_2d/linear/transform.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>

#include <cslibs_math/serialization/binary.hpp>

namespace cslibs_math {
namespace serialization {
namespace transform {
namespace binary {

template <typename T>
inline static std::size_t read(std::ifstream  &in,
                               cslibs_math_2d::Transform2<T> &transform)
{
    using transform_t = cslibs_math_2d::Transform2<T>;

    T x   = io<T>::read(in);
    T y   = io<T>::read(in);
    T yaw = io<T>::read(in);

    transform = transform_t(x,y,yaw);
    return 3ul * sizeof(T);
}

template <typename T>
inline static void  write(const cslibs_math_2d::Transform2<T> &transform,
                          std::ofstream &out)
{
    io<T>::write(transform.tx(), out);
    io<T>::write(transform.ty(), out);
    io<T>::write(transform.yaw(), out);
}

}
}
}
}

namespace YAML {
template<typename T>
struct convert<cslibs_math_2d::Transform2<T>>
{
    static Node encode(const cslibs_math_2d::Transform2<T> &rhs)
    {
        Node n;
        n["t"] = rhs.translation();
        n["r"] = rhs.yaw();
        return n;
    }
    static bool decode(const Node& n, cslibs_math_2d::Transform2<T> &rhs)
    {
        if(!n.IsMap())
            return false;
        rhs.translation() = n["t"].as<cslibs_math_2d::Vector2<T>>();
        rhs.setYaw(n["r"].as<T>());
        return true;
    }
};
}

#endif // CSLIBS_MATH_2D_SERIALIZATION_TRANSFORM_HPP
