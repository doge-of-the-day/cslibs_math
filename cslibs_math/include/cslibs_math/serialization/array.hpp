#ifndef CSLIBS_MATH_SERIALIZATION_ARRAY_HPP
#define CSLIBS_MATH_SERIALIZATION_ARRAY_HPP

#include <yaml-cpp/yaml.h>

namespace YAML {
template <typename T, std::size_t Dim>
struct convert<std::array<T, Dim>>
{
    static Node encode(const std::array<T, Dim> &rhs)
    {
        Node n;

        for (std::size_t i = 0 ; i < Dim ; ++ i)
            n.push_back(rhs[i]);

        return n;
    }

    static bool decode(const Node& n, std::array<T, Dim> &rhs)
    {
        if (!n.IsSequence() || n.size() != Dim)
            return false;

        for (std::size_t i = 0 ; i < Dim ; ++ i)
            rhs[i] = n[i].as<T>();

        return true;
    }
};
}

#endif // CSLIBS_MATH_SERIALIZATION_ARRAY_HPP
