#include <benchmark/benchmark.h>
#include <tf/tf.h>

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/utility/tiny_time.hpp>
#include <cslibs_math_2d/linear/transform.hpp>

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

static void complete_values_float(benchmark::State& state) {
  for (auto _ : state) {
    auto t = cslibs_math_2d::Transform2f::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2f{
        t.translation(), t.yaw(), t.sin(), t.cos()});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void complete_values_double(benchmark::State& state) {
  for (auto _ : state) {
    auto t = cslibs_math_2d::Transform2d::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2d{
        t.translation(), t.yaw(), t.sin(), t.cos()});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void complete_values_tf(benchmark::State& state) {
  for (auto _ : state) {
    const auto q = tf::Quaternion{0.0, 0.0, 0.0, 1.0};
    const auto v = tf::Vector3();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(tf::Transform(q,v));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void only_yaw_tf(benchmark::State& state) {
  for (auto _ : state) {
    const auto q = tf::Quaternion{0.0, 0.0, 0.0, 1.0};
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(tf::Transform(q));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void xy_float(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Vector2f::random();
    const auto x = t(0);
    const auto y = t(1);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2f{x,y});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void xy_double(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Vector2d::random();
    const auto x = t(0);
    const auto y = t(1);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2d{x,y});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void translation_float(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Vector2f::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2f{t});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void translation_double(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Vector2d::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2d{t});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void yaw_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto y = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2f{y});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void yaw_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto y = rng.get();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2d{y});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void translation_yaw_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto y = rng.get();
    const auto t = cslibs_math_2d::Vector2f::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2f{t,y});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void translation_yaw_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto y = rng.get();
    const auto t = cslibs_math_2d::Vector2d::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2d{t,y});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void xy_yaw_float(benchmark::State& state) {
  cslibs_math::random::Uniform<float, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto yaw = rng.get();
    const auto t = cslibs_math_2d::Vector2f::random();
    const auto x = t(0);
    const auto y = t(1);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2f{x,y,yaw});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void xy_yaw_double(benchmark::State& state) {
  cslibs_math::random::Uniform<double, 1> rng(-100.0, +100.0);
  for (auto _ : state) {
    const auto yaw = rng.get();
    const auto t = cslibs_math_2d::Vector2d::random();
    const auto x = t(0);
    const auto y = t(1);
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(cslibs_math_2d::Transform2d{x,y,yaw});
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void assignment_float(benchmark::State& state) {
  for (auto _ : state) {
    const auto a = cslibs_math_2d::Transform2f::random();
    auto b = cslibs_math_2d::Transform2f();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(b = a);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void assignment_double(benchmark::State& state) {
  for (auto _ : state) {
    const auto a = cslibs_math_2d::Transform2d::random();
    auto b = cslibs_math_2d::Transform2d();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(b = a);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void assignment_tf(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Transform2d::random();
    const auto a = tf::Transform(tf::createQuaternionFromYaw(t.yaw()), tf::Vector3{t.tx(), t.ty(), 0.0});
    auto b = tf::Transform();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(b = a);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void move_assignment_float(benchmark::State& state) {
  for (auto _ : state) {
    auto a = cslibs_math_2d::Transform2f::random();
    auto b = cslibs_math_2d::Transform2f();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(b = std::move(a));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void move_assignment_double(benchmark::State& state) {
  for (auto _ : state) {
    auto a = cslibs_math_2d::Transform2d::random();
    auto b = cslibs_math_2d::Transform2d();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(b = std::move(a));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void move_assignment_tf(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Transform2d::random();
    auto a = tf::Transform(tf::createQuaternionFromYaw(t.yaw()), tf::Vector3{t.tx(), t.ty(), 0.0});
    auto b = tf::Transform();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(b = std::move(a));
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}


static void inverse_float(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Transform2f::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(t.inverse());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void inverse_double(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Transform2d::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(t.inverse());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void inverse_tf(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Transform2d::random();
    const auto a = tf::Transform(tf::createQuaternionFromYaw(t.yaw()), tf::Vector3{t.tx(), t.ty(), 0.0});
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a.inverse());
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void multiply_vector_float(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Transform2f::random();
    const auto v = cslibs_math_2d::Vector2f::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(t*v);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void multiply_vector_double(benchmark::State& state) {
  for (auto _ : state) {
    const auto t = cslibs_math_2d::Transform2d::random();
    const auto v = cslibs_math_2d::Vector2d::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(t*v);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void multiply_vector_tf(benchmark::State& state) {
  for (auto _ : state) {
    const auto tm = cslibs_math_2d::Transform2d::random();
    const auto vm = cslibs_math_2d::Vector2d::random();
    const auto t = tf::Transform(tf::createQuaternionFromYaw(tm.yaw()), tf::Vector3{tm.tx(), tm.ty(), 0.0});
    const auto v = tf::Vector3{vm(0), vm(1), 0.0};

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(t * v);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}



static void multiply_transform_float(benchmark::State& state) {
  for (auto _ : state) {
    const auto a = cslibs_math_2d::Transform2f::random();
    const auto b = cslibs_math_2d::Transform2f::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a*b);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void multiply_transform_double(benchmark::State& state) {
  for (auto _ : state) {
    const auto a = cslibs_math_2d::Transform2d::random();
    const auto b = cslibs_math_2d::Transform2d::random();
    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a*b);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

static void multiply_transform_tf(benchmark::State& state) {
  for (auto _ : state) {
    const auto ta = cslibs_math_2d::Transform2d::random();
    const auto tb = cslibs_math_2d::Transform2d::random();
    const auto a = tf::Transform(tf::createQuaternionFromYaw(ta.yaw()), tf::Vector3{ta.tx(), ta.ty(), 0.0});
    const auto b = tf::Transform(tf::createQuaternionFromYaw(tb.yaw()), tf::Vector3{tb.tx(), tb.ty(), 0.0});

    auto start = cslibs_math::utility::tiny_time::clock_t::now();
    benchmark::DoNotOptimize(a * b);
    auto elapsed_seconds =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            cslibs_math::utility::tiny_time::clock_t::now() - start);

    state.SetIterationTime(elapsed_seconds.count());
  }
}

BENCHMARK(default_float);
BENCHMARK(default_double);
BENCHMARK(default_tf);
BENCHMARK(complete_values_float);
BENCHMARK(complete_values_double);
BENCHMARK(complete_values_tf);
BENCHMARK(only_yaw_tf);
BENCHMARK(xy_float);
BENCHMARK(xy_double);
BENCHMARK(translation_float);
BENCHMARK(translation_double);
BENCHMARK(yaw_float);
BENCHMARK(yaw_double);
BENCHMARK(translation_yaw_float);
BENCHMARK(translation_yaw_double);
BENCHMARK(xy_yaw_float);
BENCHMARK(xy_yaw_double);
BENCHMARK(assignment_float);
BENCHMARK(assignment_double);
BENCHMARK(assignment_tf);
BENCHMARK(move_assignment_float);
BENCHMARK(move_assignment_double);
BENCHMARK(move_assignment_tf);
BENCHMARK(inverse_float);
BENCHMARK(inverse_double);
BENCHMARK(inverse_tf);
BENCHMARK(multiply_vector_float);
BENCHMARK(multiply_vector_double);
BENCHMARK(multiply_vector_tf);
BENCHMARK(multiply_transform_float);
BENCHMARK(multiply_transform_double);
BENCHMARK(multiply_transform_tf);


int main(int argc, char** argv) {
  ::benchmark::Initialize(&argc, argv);
  if (::benchmark::ReportUnrecognizedArguments(argc, argv)) return 1;
  ::benchmark::RunSpecifiedBenchmarks();
  std::cout << "Byte-Sizes of types : \n"
            << "  tf::Transform " << sizeof(tf::Transform) << "\n"
            << "  Transform2f   " << sizeof(cslibs_math_2d::Transform2f) << "\n"
            << "  Transform2d   " << sizeof(cslibs_math_2d::Transform2d)
            << std::endl;
  return 0;
}
