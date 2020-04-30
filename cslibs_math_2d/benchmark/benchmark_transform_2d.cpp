#include <benchmark/benchmark.h>

#include <tf/tf.h>

#include <cslibs_math_2d/linear/transform.hpp>
#include <cslibs_math/random/random.hpp>
#include <cslibs_math/utility/tiny_time.hpp>

static void default_float(benchmark::State& state) {
  for (auto _ : state) {
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2f());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void default_double(benchmark::State& state) {
  for (auto _ : state) {
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2d());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void default_tf(benchmark::State& state) {
  for (auto _ : state) {
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(tf::Transform());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

BENCHMARK(default_float);
BENCHMARK(default_double);
BENCHMARK(default_tf);

BENCHMARK_MAIN()