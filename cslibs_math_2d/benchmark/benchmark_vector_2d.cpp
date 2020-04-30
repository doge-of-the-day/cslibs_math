#include <benchmark/benchmark.h>
#include <tf/tf.h>

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/utility/tiny_time.hpp>
#include <cslibs_math_2d/linear/vector.hpp>

static void default_float(benchmark::State& state) {
  for (auto _ : state) {
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Vector2f());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void default_double(benchmark::State& state) {
  for (auto _ : state) {
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Vector2d());
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
    benchmark::DoNotOptimize(cslibs_math_2d::Vector2f(x));
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
    benchmark::DoNotOptimize(cslibs_math_2d::Vector2d(x));
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
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Vector2f(x, y));
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
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Vector2d(x, y));
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
    const auto z = 0.0;
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(tf::Vector3(x, y, z));
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
    const cslibs_math_2d::Vector2f v(x, y);
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
    const cslibs_math_2d::Vector2d v(x, y);
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
    const auto z = 0.0;
    tf::Vector3 v(x, y, z);
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
    const cslibs_math_2d::Vector2f v(x, y);
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
    const cslibs_math_2d::Vector2d v(x, y);
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
    const auto z = 0.0;
    tf::Vector3 v(x, y, z);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(v.length2());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void add_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const cslibs_math_2d::Vector2f a(rng.get(), rng.get());
    const cslibs_math_2d::Vector2f b(rng.get(), rng.get());

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a + b);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void add_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const cslibs_math_2d::Vector2d a(rng.get(), rng.get());
    const cslibs_math_2d::Vector2d b(rng.get(), rng.get());

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a + b);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void add_tf(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto z = 0.0;
    const tf::Vector3 a(rng.get(), rng.get(), z);
    const tf::Vector3 b(rng.get(), rng.get(), z);

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a + b);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void sub_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const cslibs_math_2d::Vector2f a(rng.get(), rng.get());
    const cslibs_math_2d::Vector2f b(rng.get(), rng.get());

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a - b);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void sub_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const cslibs_math_2d::Vector2d a(rng.get(), rng.get());
    const cslibs_math_2d::Vector2d b(rng.get(), rng.get());

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a - b);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void sub_tf(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto z = 0.0;
    const tf::Vector3 a(rng.get(), rng.get(), z);
    const tf::Vector3 b(rng.get(), rng.get(), z);

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a - b);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void dot_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const cslibs_math_2d::Vector2f a(rng.get(), rng.get());
    const cslibs_math_2d::Vector2f b(rng.get(), rng.get());

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a.dot(b));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void dot_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const cslibs_math_2d::Vector2d a(rng.get(), rng.get());
    const cslibs_math_2d::Vector2d b(rng.get(), rng.get());

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a.dot(b));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void dot_tf(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);

  for (auto _ : state) {
    const auto z = 0.0;
    const tf::Vector3 a(rng.get(), rng.get(), z);
    const tf::Vector3 b(rng.get(), rng.get(), z);

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a.dot(b));
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
BENCHMARK(add_float);
BENCHMARK(add_double);
BENCHMARK(add_tf);
BENCHMARK(sub_float);
BENCHMARK(sub_double);
BENCHMARK(sub_tf);
BENCHMARK(dot_float);
BENCHMARK(dot_double);
BENCHMARK(dot_tf);

BENCHMARK_MAIN()