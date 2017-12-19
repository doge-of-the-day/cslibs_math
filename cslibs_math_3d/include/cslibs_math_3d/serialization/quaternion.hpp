#ifndef CSLIBS_MATH_3D_SERIALIZATION_QUATERNION_HPP
#define CSLIBS_MATH_3D_SERIALIZATION_QUATERNION_HPP

#include <cslibs_math_3d/linear/quaternion.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {
template<>
struct convert<cslibs_math_3d::Quaternion>
{
    static Node encode(const cslibs_math_3d::Quaternion &rhs)
    {
        Node n;
        n.push_back(rhs.x());
        n.push_back(rhs.y);
        n.push_back(rhs.z());
        n.push_back(rhs.w());
        return n;
    }
    static bool decode(const Node& n, cslibs_math_3d::Quaternion &rhs)
    {
        if(!n.IsSequence() || n.size() != 4)
            return false;

        rhs.x() = n[0].as<double>();
        rhs.y() = n[0].as<double>();
        rhs.z() = n[0].as<double>();
        rhs.w() = n[0].as<double>();
        return true;
    }
};
}

#endif // CSLIBS_MATH_3D_SERIALIZATION_QUATERNION_HPP
