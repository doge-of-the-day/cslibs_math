#include <benchmark/benchmark.h>

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/utility/tiny_time.hpp>
#include <cslibs_math/common/angle.hpp>

static void normalize_std_atan2(benchmark::State& state) {
  const auto normalize = [](const double x) {
    return std::atan2(std::sin(x), std::cos(x));
  };

  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(normalize(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void normalize_atan2(benchmark::State& state) {
  const auto normalize = [](const double x) {
    return atan2(sin(x), cos(x));
  };

  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(normalize(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void normalize_while(benchmark::State& state) {
  const auto normalize = [](const double x)
  {
    double y = x;
    while (y <= -2.0 * M_PI) {
      y += M_PI;
    }
    while (y > 2.0 * M_PI) {
      y -= M_PI;
    }
    return y;
 };

  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(normalize(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void normalize_cslibs_math(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math::common::angle::normalize(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

BENCHMARK(normalize_std_atan2);
BENCHMARK(normalize_atan2);
BENCHMARK(normalize_while);
BENCHMARK(normalize_cslibs_math);

BENCHMARK_MAIN()