#include <benchmark/benchmark.h>

#include <tf/tf.h>

#include <cslibs_math_3d/linear/vector.hpp>
#include <cslibs_math/random/random.hpp>
#include <cslibs_math/utility/tiny_time.hpp>

static void default_float(benchmark::State& state) {
  for (auto _ : state) {
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_3d::Vector3f());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void default_double(benchmark::State& state) {
  for (auto _ : state) {
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_3d::Vector3d());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void default_tf(benchmark::State& state) {
  for (auto _ : state) {
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(tf::Vector3());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}


static void single_constant_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_3d::Vector3f(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void single_constant_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_3d::Vector3d(x));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}


static void constant_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    const auto y = rng.get();
    const auto z = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_3d::Vector3f(x,y,z));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void constant_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    const auto y = rng.get();
    const auto z = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_3d::Vector3d(x,y,z));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void constant_tf(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    const auto y = rng.get();
    const auto z = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(tf::Vector3(x,y,z));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void norm_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    const auto y = rng.get();
    const auto z = rng.get();
    const cslibs_math_3d::Vector3f v(x,y,z);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(v.length());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void norm_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    const auto y = rng.get();
    const auto z = rng.get();
    const cslibs_math_3d::Vector3d v(x,y,z);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(v.length());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void norm_tf(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    const auto y = rng.get();
    const auto z = rng.get();
    tf::Vector3 v(x,y,z);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(v.length());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void norm2_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    const auto y = rng.get();
    const auto z = rng.get();
    const cslibs_math_3d::Vector3f v(x,y,z);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(v.length2());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void norm2_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    const auto y = rng.get();
    const auto z = rng.get();
    const cslibs_math_3d::Vector3d v(x,y,z);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(v.length2());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void norm2_tf(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto x = rng.get();
    const auto y = rng.get();
    const auto z = rng.get();
    tf::Vector3 v(x,y,z);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(v.length2());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

BENCHMARK(default_float);
BENCHMARK(default_double);
BENCHMARK(default_tf);
BENCHMARK(single_constant_float);
BENCHMARK(single_constant_double);
BENCHMARK(constant_float);
BENCHMARK(constant_double);
BENCHMARK(constant_tf);
BENCHMARK(norm_float);
BENCHMARK(norm_double);
BENCHMARK(norm_tf);
BENCHMARK(norm2_float);
BENCHMARK(norm2_double);
BENCHMARK(norm2_tf);

BENCHMARK_MAIN();
