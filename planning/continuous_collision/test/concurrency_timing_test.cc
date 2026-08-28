// The two per-call parallel *scaling* claims, split out of concurrency_test.cc
// because they are wall-clock claims and it is not.
//
// Every case in concurrency_test.cc is an equality and runs under every build
// flavor. A duration, by contrast, means nothing under an instrumented build:
// Valgrind serializes threads outright, so `parallel < serial` inverts and the
// case fails for a reason that has nothing to do with the driver. Hence the
// separate target, which carries disable_in_compilation_mode_dbg and the
// no_valgrind_tools tag, and hence TimingClaimsAreMeaningless() below, which
// skips whatever the build tags did not already exclude.
//
// The corpus, the deep workload and the option defaults are shared with
// concurrency_test.cc through concurrency_test_utilities.h.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>

#include <gtest/gtest.h>

#include "drake/common/parallelism.h"
#include "drake/planning/continuous_collision/test/concurrency_test_utilities.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace test {
namespace {

// Per-call parallel scaling: the two properties the driver in
// certifier_internal.cc exists for.
//
//   a) a deep tree inside a single segment spreads over the workers, instead
//      of sitting behind one fixed seed that no other worker can split;
//   b) a check too small to pay for workers never loses by being asked for
//      them, which matters because Parallelism::Max() is the *default* value
//      of Options::parallelism.
//
// Both are timing claims, so both are written to survive a loaded machine: a
// ratio with a wide margin, best-of-three, and a skip when the hardware or the
// build cannot support the claim at all. They are regression detectors rather
// than benchmarks, and should only ever fire on a driver that has stopped
// distributing work.

// True when the build cannot support a meaningful wall-clock claim: a sanitizer
// build serializes and inflates everything, an unoptimized build changes the
// ratios, and fewer than eight hardware threads means there is no parallelism
// to measure.
//
// The compile-time tests below only see the sanitizers this translation unit
// was itself instrumented with. Valgrind instruments nothing at compile time,
// and a sanitizer runtime linked in from elsewhere is equally invisible, so the
// environment is consulted too: the tools that make a duration meaningless all
// announce themselves through an options variable. That is the same test
// limit_malloc.cc uses to disarm itself, and the same VALGRIND_OPTS check
// gcs_trajectory_optimization_test.cc uses.
bool TimingClaimsAreMeaningless() {
#if defined(__SANITIZE_THREAD__) || defined(__SANITIZE_ADDRESS__)
  return true;
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer) || __has_feature(address_sanitizer)
  return true;
#endif
#endif
#ifndef NDEBUG
  return true;
#else
  for (const char* variable : {"VALGRIND_OPTS", "ASAN_OPTIONS", "LSAN_OPTIONS",
                               "TSAN_OPTIONS", "UBSAN_OPTIONS"}) {
    if (std::getenv(variable) != nullptr) return true;
  }
  return std::thread::hardware_concurrency() < 8;
#endif
}

template <typename F>
double BestOfThreeSeconds(F&& body) {
  body();  // Warm up: first-touch page faults, Drake's own lazy caches.
  double best = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    const auto start = std::chrono::steady_clock::now();
    body();
    best = std::min(best, std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - start)
                              .count());
  }
  return best;
}

GTEST_TEST(ConcurrencyTest, DeepWorkloadIsFasterInParallel) {
  if (TimingClaimsAreMeaningless()) GTEST_SKIP();
  const DeepWorkload& deep = Deep();
  ASSERT_NE(deep.entry, nullptr);
  const BezierCurve<double> trajectory = deep.entry->trajectory();
  const Options serial_options = deep.options(Parallelism::None());
  const Options parallel_options = deep.options(Parallelism(8));

  const double serial = BestOfThreeSeconds([&]() {
    deep.entry->checker->CheckTrajectory(trajectory, serial_options);
  });
  const double parallel = BestOfThreeSeconds([&]() {
    deep.entry->checker->CheckTrajectory(trajectory, parallel_options);
  });
  std::cout << "\n[ concurrency ] deep workload: serial " << 1e3 * serial
            << " ms, Parallelism(8) " << 1e3 * parallel << " ms ("
            << serial / parallel << "x)\n\n";
  // 1.43x is the bound that separates a driver that distributes deep work from
  // one that leaves it on a single worker, without being a performance
  // assertion in disguise.
  EXPECT_LT(parallel, 0.7 * serial);
}

GTEST_TEST(ConcurrencyTest, SmallCheckIsNotSlowerInParallel) {
  if (TimingClaimsAreMeaningless()) GTEST_SKIP();
  // A two-waypoint edge in one of the corpus worlds is the small check: a
  // handful of nodes, dominated by the serial breakpoint pass. Asked for the
  // default Parallelism::Max(), the driver must decline to hire anyone rather
  // than pay a worker-startup bill several times the size of the work.
  const Case& entry = *Corpus().front();
  const VectorXd q1 = entry.control_points.col(0);
  const VectorXd q2 = entry.control_points.rightCols(1);
  const Options serial_options =
      BaseOptions(Parallelism::None(), SearchMode::kCertifyAll);
  const Options parallel_options =
      BaseOptions(Parallelism::Max(), SearchMode::kCertifyAll);
  ASSERT_LT(entry.checker->CheckEdge(q1, q2, serial_options).stats.nodes, 100u);

  const double serial = BestOfThreeSeconds([&]() {
    entry.checker->CheckEdge(q1, q2, serial_options);
  });
  const double parallel = BestOfThreeSeconds([&]() {
    entry.checker->CheckEdge(q1, q2, parallel_options);
  });
  std::cout << "\n[ concurrency ] small check: serial " << 1e3 * serial
            << " ms, Parallelism::Max() " << 1e3 * parallel << " ms ("
            << serial / parallel << "x)\n\n";
  // The driver never hires for a check this small, so the two paths run the
  // same code and parity is what to expect; the 1.5x bound leaves room for
  // scheduler noise on a loaded machine without admitting a real slowdown.
  EXPECT_LT(parallel, 1.5 * serial);
}

}  // namespace
}  // namespace test
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
