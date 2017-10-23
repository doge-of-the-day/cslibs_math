#ifndef NORMAL_SAMPLER_HPP
#define NORMAL_SAMPLER_HPP

#include <memory>
#include <cslibs_math/random/random.hpp>
#include "traits.hpp"

namespace muse_smc {
namespace state_space_samplers {
template<typename... Types>
class Normal {
public:
    typedef std::shared_ptr<Normal> Ptr;

    static_assert(sizeof...(Types) > 0, "Constraint : Dimension > 0");
    static_assert(is_valid_type<Types...>::value, "Parameter list contains forbidden type!");

    static const std::size_t Dimension = sizeof...(Types);

    using rng_t = cslibs_math::random::Normal<Dimension>;

    Normal() = delete;
    Normal(const Normal &other) = delete;

    Normal(const typename rng_t::Vector &pose,
           const typename rng_t::Matrix &covariance,
           const unsigned int seed = 0) :
        rng_(pose, covariance, seed)
    {
    }

    inline typename rng_t::Vector get()
    {
        typename rng_t::Vector sample = rng_.get();
        Arguments<Dimension, typename rng_t::Vector, Types...>::normalize(sample);
        return sample;
    }

private:
    rng_t rng_;
};
}
}

#endif /* NORMAL_SAMPLER_HPP */
