#ifndef CSLIBS_MATH_SERIALIZATION_WEIGHTED_DISTRIBUTION_HPP
#define CSLIBS_MATH_SERIALIZATION_WEIGHTED_DISTRIBUTION_HPP

#include <cslibs_math/statistics/weighted_distribution.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>

#include <cslibs_math/serialization/binary.hpp>

namespace cslibs_math {
namespace serialization {
namespace weighted_distribution {
template <std::size_t Dim, std::size_t lambda_ratio_exponent>
struct binary {
    using distribution_t    = cslibs_math::statistics::WeightedDistribution<Dim, lambda_ratio_exponent>;
    using sample_t          = typename distribution_t::sample_t;
    using correlated_t      = typename distribution_t::covariance_t;

    static const std::size_t size = sizeof(std::size_t) + 2 * sizeof(double) + Dim * sizeof(double) + Dim * Dim * sizeof(double);

    inline static std::size_t read(std::ifstream  &in,
                                   distribution_t &distribution)
    {
        sample_t     mean;
        correlated_t corr;

        std::size_t sc = io<std::size_t>::read(in);
        double w       = io<double>::read(in);
        double w_sq    = io<double>::read(in);

        for(std::size_t i = 0 ; i < Dim ; ++i)
            mean(i) = io<double>::read(in);

        for(std::size_t i = 0 ; i < Dim; ++i) {
            for(std::size_t j = 0 ; j < Dim ; ++j) {
                corr(i,j) = io<double>::read(in);
            }
        }
        distribution = distribution_t(sc, w, w_sq, mean, corr);

        return size;
    }

    inline static void write(std::ofstream &out)
    {
        io<std::size_t>::write(0, out);
        io<double>::write(0, out);
        io<double>::write(0, out);

        for(std::size_t i = 0 ; i < Dim ; ++i)
            io<double>::write(0.0, out);

        for(std::size_t i = 0 ; i < Dim; ++i) {
            for(std::size_t j = 0 ; j < Dim ; ++j) {
                io<double>::write(0.0, out);
            }
        }
    }

    inline static void write(const distribution_t &distribution,
                             std::ofstream &out)
    {
        const sample_t      mean = distribution.getMean();
        const correlated_t  corr = distribution.getCorrelated();
        const std::size_t   sc   = distribution.getSampleCount();
        const double        w    = distribution.getWeight();
        const double        w_sq = distribution.getWeightSQ();

        io<std::size_t>::write(sc, out);
        io<double>::write(w, out);
        io<double>::write(w_sq, out);

        for(std::size_t i = 0 ; i < Dim ; ++i)
            io<double>::write(mean(i), out);

        for(std::size_t i = 0 ; i < Dim; ++i) {
            for(std::size_t j = 0 ; j < Dim ; ++j) {
                io<double>::write(corr(i,j), out);
            }
        }
    }
};
}
}
}

namespace YAML {
template<std::size_t Dim, std::size_t lambda_ratio_exponent>
struct convert<cslibs_math::statistics::WeightedDistribution<Dim, lambda_ratio_exponent>>
{
    using sample_t     = typename cslibs_math::statistics::WeightedDistribution<Dim, lambda_ratio_exponent>::sample_t;
    using covariance_t = typename cslibs_math::statistics::WeightedDistribution<Dim, lambda_ratio_exponent>::covariance_t;

    static Node encode(const cslibs_math::statistics::WeightedDistribution<Dim, lambda_ratio_exponent> &rhs)
    {
        Node n;
        n.push_back(rhs.getSampleCount());
        n.push_back(rhs.getWeight());
        n.push_back(rhs.getWeightSQ());

        sample_t mean = rhs.getMean();
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            n.push_back(mean(i));

        covariance_t correlated = rhs.getCorrelated();
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            for (std::size_t j = 0 ; j < Dim ; ++ j)
                n.push_back(correlated(i, j));

        return n;
    }

    static bool decode(const Node& n, cslibs_math::statistics::WeightedDistribution<Dim, lambda_ratio_exponent> &rhs)
    {
        if (!n.IsSequence() || n.size() != (3 + Dim + Dim * Dim))
            return false;

        std::size_t p = 0;
        std::size_t sc = n[p++].as<std::size_t>();
        double w       = n[p++].as<double>();
        double w_sq    = n[p++].as<double>();

        sample_t mean(sample_t::Zero());
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            mean(i) = n[p++].as<double>();

        covariance_t correlated(covariance_t::Zero());
        for (std::size_t i = 0 ; i < Dim ; ++ i)
            for (std::size_t j = 0 ; j < Dim ; ++ j)
                correlated(i, j) = n[p++].as<double>();

        rhs = cslibs_math::statistics::WeightedDistribution<Dim, lambda_ratio_exponent>(sc, w, w_sq, mean, correlated);
        return true;
    }
};

template<std::size_t lambda_ratio_exponent>
struct convert<cslibs_math::statistics::WeightedDistribution<1, lambda_ratio_exponent>>
{
    static Node encode(const cslibs_math::statistics::WeightedDistribution<1, lambda_ratio_exponent> &rhs)
    {
        Node n;
        n.push_back(rhs.getSampleCount());
        n.push_back(rhs.getWeight());
        n.push_back(rhs.getWeightSQ());
        n.push_back(rhs.getMean());
        n.push_back(rhs.getSquared());

        return n;
    }

    static bool decode(const Node& n, cslibs_math::statistics::WeightedDistribution<1, lambda_ratio_exponent> &rhs)
    {
        if(!n.IsSequence() || n.size() != 5)
            return false;

        rhs = cslibs_math::statistics::WeightedDistribution<1, lambda_ratio_exponent>(
                    n[0].as<std::size_t>(), n[1].as<double>(), n[2].as<double>(), n[3].as<double>(), n[4].as<double>());
        return true;
    }
};
}

#endif // CSLIBS_MATH_SERIALIZATION_WEIGHTED_DISTRIBUTION_HPP
