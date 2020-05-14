#include <benchmark/benchmark.h>

#include <iostream>

#include <cslibs_math/common/equal.hpp>
#include <cslibs_math/approx/fractional.hpp>
#include <cslibs_math/random/random.hpp>
#include <cslibs_math/utility/tiny_time.hpp>
#include <iomanip>

using Fractionald = cslibs_math::approx::Fractional<double>;

static void fractional_constructor_default(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(0.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(Fractionald(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

const auto SAMPLES = 100000;
const auto SAMPLING_ITERATIONS = 30;

static void applicationFractional(benchmark::State& state) {
  auto task = [&state]() {
    cslibs_math::random::Uniform<double, 1> rng(0.0, 1.0);
    cslibs_math::utility::tiny_time::duration_t elapsed;

    const auto sample = [&rng]() {
      Fractionald s(1.0);
      for(auto i = 0 ; i < SAMPLING_ITERATIONS ; ++i){
        s = s * Fractionald(rng.get());
      }
      return s;
    };

    Fractionald sum;
    std::vector<Fractionald> samples;
    for (auto i = 0; i < SAMPLES; ++i) {
      const auto x = sample();
      auto start = cslibs_math::utility::tiny_time::clock_t::now();
      auto& sample = samples.emplace_back(Fractionald(x));
      sum = sum + sample;
      elapsed += cslibs_math::utility::tiny_time::clock_t::now() - start;
    }
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    for (auto& s : samples) {
      s = s / sum;
    }
    elapsed += cslibs_math::utility::tiny_time::clock_t::now() - start;

    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(elapsed);
    state.SetIterationTime(elapsed_seconds.count());

    double normalized_sum{0};
    for(const auto &s : samples){
      normalized_sum += s.value();
    }
    return normalized_sum;
  };

  for (auto _ : state) {
    const auto res = task();
    if(cslibs_math::common::eq(1.0, res)) {
      std::cerr << "There went something wrong in applicationFractional " << std::setprecision(20) << res << std::endl;
    }
  }
}

static void applicationLog(benchmark::State& state) {
  auto task = [&state]() {
    cslibs_math::random::Uniform<double, 1> rng(0.0, 1.0);
    cslibs_math::utility::tiny_time::duration_t elapsed;

    const auto add = [](const double x, const double y)
    {
      /// x and y are log values
      return x + std::log1p(std::exp(y - x));
    };

    const auto sample = [&rng]() {
      auto s(1.0);
      for(auto i = 0 ; i < SAMPLING_ITERATIONS ; ++i){
        s += std::log(rng.get());
      }
      return s;
    };

    double sum{0};
    std::vector<double> samples;
    for (auto i = 0; i < SAMPLES; ++i) {
      const auto x = sample();
      auto start = cslibs_math::utility::tiny_time::clock_t::now();
      auto& sample = samples.emplace_back(std::log(x));
      sum = add(sum, sample);
      elapsed += cslibs_math::utility::tiny_time::clock_t::now() - start;
    }
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    for (auto& s : samples) {
      s -= sum;
    }
    elapsed += cslibs_math::utility::tiny_time::clock_t::now() - start;

    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(elapsed);
    state.SetIterationTime(elapsed_seconds.count());

    double normalized_sum{0};
    for(const auto &s : samples){
      normalized_sum += std::exp(s);
    }
    return normalized_sum;
  };

  for (auto _ : state) {
    const auto res = task();
    if(cslibs_math::common::eq(1.0, res)) {
      std::cerr << "There went something wrong in applicationLog " << std::setprecision(20) << res << std::endl;
    }
  }
}

static void applicationDefault(benchmark::State& state) {
  auto task = [&state]() {
    cslibs_math::random::Uniform<double, 1> rng(0.0, 1.0);
    cslibs_math::utility::tiny_time::duration_t elapsed;

    const auto sample = [&rng]() {
      auto s(1.0);
      for(auto i = 0 ; i < SAMPLING_ITERATIONS ; ++i){
        s *= rng.get();
      }
      return s;
    };

    double sum{0};
    std::vector<double> samples;
    for (auto i = 0; i < SAMPLES; ++i) {
      const auto x = sample();
      auto start = cslibs_math::utility::tiny_time::clock_t::now();
      auto& sample = samples.emplace_back(x);
      sum += sample;
      elapsed += cslibs_math::utility::tiny_time::clock_t::now() - start;
    }
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    for (auto& s : samples) {
      s /= sum;
    }
    elapsed += cslibs_math::utility::tiny_time::clock_t::now() - start;

    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(elapsed);
    state.SetIterationTime(elapsed_seconds.count());

    double normalized_sum{0};
    for(const auto &s : samples){
      normalized_sum += s;
    }
    return normalized_sum;
  };

  for (auto _ : state) {
    const auto res = task();
    if(cslibs_math::common::eq(1.0, res)) {
      std::cerr << "There went something wrong in applicationLog " << std::setprecision(20) << res << std::endl;
    }
  }
}


BENCHMARK(fractional_constructor_default);
BENCHMARK(applicationFractional);
BENCHMARK(applicationLog);
BENCHMARK(applicationDefault);

BENCHMARK_MAIN()