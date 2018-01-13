#ifndef CSLIBS_MATH_SERIALIZATION_ARRAY_HPP
#define CSLIBS_MATH_SERIALIZATION_ARRAY_HPP

#include <yaml-cpp/yaml.h>

template<typename T, std::size_t Dim>
inline YAML::Emitter& operator << (YAML::Emitter& out, const std::array<T, Dim>& a) {
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for(const T& e : a) {
        out << e;
    }
    out << YAML::EndSeq;
    return out;
}

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
