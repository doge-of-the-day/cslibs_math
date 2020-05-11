#include <benchmark/benchmark.h>

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/approx/exp.hpp>
#include <cslibs_math/utility/tiny_time.hpp>


static void std_exp(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(std::exp(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void std_exp2(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(std::exp2(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}


static void c_exp(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(exp(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void approx_exp_8(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math::approx::exp<8, double>(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void approx_exp_10(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math::approx::exp<10, double>(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}


static void approx_exp_20(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math::approx::exp<20, double>(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void approx_exp_40(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math::approx::exp<40, double>(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}


static void std_log(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(std::log(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}


static void std_log2(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(std::log2(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void c_log(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(log(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

BENCHMARK(std_exp);
BENCHMARK(std_exp2);
BENCHMARK(approx_exp_8);
BENCHMARK(approx_exp_10);
BENCHMARK(approx_exp_20);
BENCHMARK(approx_exp_40);
BENCHMARK(c_exp);
BENCHMARK(std_log);
BENCHMARK(std_log2);
BENCHMARK(c_log);

BENCHMARK_MAIN()