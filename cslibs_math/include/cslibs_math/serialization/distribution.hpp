#ifndef CSLIBS_MATH_SERIALIZATION_DISTRIBUTION_HPP
#define CSLIBS_MATH_SERIALIZATION_DISTRIBUTION_HPP

#include <cslibs_math/statistics/distribution.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {
template<std::size_t Dim, std::size_t lambda_ratio_exponent>
struct convert<cslibs_math::statistics::Distribution<Dim, lambda_ratio_exponent>>
{
    using sample_t     = typename cslibs_math::statistics::Distribution<Dim, lambda_ratio_exponent>::sample_t;
    using covariance_t = typename cslibs_math::statistics::Distribution<Dim, lambda_ratio_exponent>::covariance_t;

    static Node encode(const cslibs_math::statistics::Distribution<Dim, lambda_ratio_exponent> &rhs)
    {
        Node n;
        n.push_back(rhs.getN());

        sample_t mean = rhs.getMean();
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            n.push_back(mean(i));

        covariance_t correlated = rhs.getCorrelated();
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            for (std::size_t j = 0 ; j < Dim ; ++ j)
                n.push_back(correlated(i, j));

        return n;
    }

    static bool decode(const Node& n, cslibs_math::statistics::Distribution<Dim, lambda_ratio_exponent> &rhs)
    {
        if (!n.IsSequence() || n.size() != (1 + Dim + Dim * Dim))
            return false;

        std::size_t p = 0;
        std::size_t num = n[p++].as<std::size_t>();

        sample_t mean(sample_t::Zero());
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            mean(i) = n[p++].as<double>();

        covariance_t correlated(covariance_t::Zero());
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            for (std::size_t j = 0 ; j < Dim ; ++ j)
                correlated(i, j) = n[p++].as<double>();

        rhs = cslibs_math::statistics::Distribution<Dim, lambda_ratio_exponent>(num, mean, correlated);
        return true;
    }
};

template<std::size_t lambda_ratio_exponent>
struct convert<cslibs_math::statistics::Distribution<1, lambda_ratio_exponent>>
{
    static Node encode(const cslibs_math::statistics::Distribution<1, lambda_ratio_exponent> &rhs)
    {
        Node n;
        n.push_back(rhs.getN());
        n.push_back(rhs.getMean());
        n.push_back(rhs.getSquared());

        return n;
    }

    static bool decode(const Node& n, cslibs_math::statistics::Distribution<1, lambda_ratio_exponent> &rhs)
    {
        if(!n.IsSequence() || n.size() != 3)
            return false;

        rhs = cslibs_math::statistics::Distribution<1, lambda_ratio_exponent>(
                    n[0].as<std::size_t>(), n[1].as<double>(), n[2].as<double>());
        return true;
    }
};
}

#endif // CSLIBS_MATH_SERIALIZATION_DISTRIBUTION_HPP
