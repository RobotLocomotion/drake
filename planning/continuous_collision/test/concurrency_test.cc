// Concurrency determinism, on the fixed corpus of ten random cases (a mix of
// free and violating) that test_utilities.h builds, plus the deep workload it
// derives from that corpus:
//
//   1. The answer does not depend on the thread count. Verdict and earliest
//      witness are identical at Parallelism {1, 2, 8, 16}, and on a case that
//      certifies free (where the branch-and-bound bound never tightens and the
//      whole tree is explored) so is Result::num_nodes.
//   2. The public Check* methods are safe to call concurrently on one instance.
//   3. The deep workload, which unlike any corpus case is big enough that the
//      driver actually hires helpers, explores the same tree and reports the
//      same result at every thread count, concurrent callers included.
//
// Every case is an equality, not a wall-clock claim, so this target runs under
// every build flavor. This is the test to run under ThreadSanitizer:
//
//    bazel test --config=tsan //planning/continuous_collision:concurrency_test
//
// On recent kernels the default `vm.mmap_rnd_bits` puts mappings outside the
// range TSan's shadow memory expects and the runtime aborts before main ever
// runs; run the binary under `setarch $(uname -m) -R`, or lower
// vm.mmap_rnd_bits to 28. A report rooted in a continuous_collision frame is a
// real bug; one rooted entirely in Drake belongs in a suppression file.

#include <cstddef>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/parallelism.h"
#include "drake/planning/continuous_collision/test/test_utilities.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace test {
namespace {

// Verdict and witness always; the node count too whenever the run certified,
// because then nothing was pruned and both runs walked the identical tree.
::testing::AssertionResult SameResult(const Result& a, const Result& b) {
  if (a.verdict != b.verdict) {
    return ::testing::AssertionFailure() << "verdicts differ";
  }
  if (a.verdict == Verdict::kCertifiedFree && a.num_nodes != b.num_nodes) {
    return ::testing::AssertionFailure()
           << "node counts differ: " << a.num_nodes << " vs " << b.num_nodes;
  }
  return FindingIdentical(a.finding, b.finding);
}

GTEST_TEST(ConcurrencyTest, CorpusIsBalanced) {
  const auto& corpus = Corpus();
  ASSERT_EQ(static_cast<int>(corpus.size()), kNumCases);
  int free_count = 0;
  int violating_count = 0;
  for (const auto& entry : corpus) {
    (entry->serial_verdict == Verdict::kCertifiedFree ? free_count
                                                      : violating_count) += 1;
  }
  EXPECT_GE(free_count, kMinFreeCases);
  EXPECT_GE(violating_count, kMinViolatingCases);
}

GTEST_TEST(ConcurrencyTest, ThreadCountInvariantAndSeriallyDeterministic) {
  for (const auto& entry : Corpus()) {
    const BezierCurve<double> trajectory = entry->trajectory();
    const Options serial_options = BaseOptions(Parallelism::None());
    const Result serial =
        entry->checker->CheckTrajectory(trajectory, serial_options);
    SCOPED_TRACE(entry->name);
    // Serial runs are bit-deterministic.
    EXPECT_TRUE(SameResult(
        serial, entry->checker->CheckTrajectory(trajectory, serial_options)));
    for (const int threads : {2, 8, 16}) {
      SCOPED_TRACE("threads " + std::to_string(threads));
      EXPECT_TRUE(SameResult(
          serial, entry->checker->CheckTrajectory(
                      trajectory, BaseOptions(Parallelism(threads)))));
    }
  }
}

GTEST_TEST(ConcurrencyTest, ConcurrentCallsOnOneCheckerMatchSequential) {
  // Every worker hits the *same* checker object through all three public entry
  // points, so they contend for the construction-time context pool; the lease
  // must hand each call its own contexts. Each worker also asks for internal
  // parallelism, so the pool is under pressure from both directions at once.
  const auto& corpus = Corpus();
  const Options options = BaseOptions(Parallelism(2));

  std::vector<Result> trajectory_expected;
  std::vector<Result> edge_expected;
  std::vector<Result> path_expected;
  std::vector<Eigen::MatrixXd> waypoints;
  for (const auto& entry : corpus) {
    const VectorXd q1 = entry->control_points.col(0);
    const VectorXd q2 = entry->control_points.rightCols(1);
    Eigen::MatrixXd w(q1.size(), 3);
    w.col(0) = q1;
    w.col(1) = 0.5 * (q1 + q2);
    w.col(2) = q2;
    waypoints.push_back(w);
    trajectory_expected.push_back(
        entry->checker->CheckTrajectory(entry->trajectory(), options));
    edge_expected.push_back(entry->checker->CheckEdge(q1, q2, options));
    path_expected.push_back(entry->checker->CheckPath(w, options));
  }

  constexpr int kThreads = 8;
  // gtest assertions are not safe off the main thread, so each worker counts
  // its own mismatches into its own slot and the main thread asserts after the
  // join.
  std::vector<int> mismatches(kThreads, 0);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int r = 0; r < 3; ++r) {
        for (std::size_t k = 0; k < corpus.size(); ++k) {
          const Case& entry = *corpus[k];
          const VectorXd q1 = entry.control_points.col(0);
          const VectorXd q2 = entry.control_points.rightCols(1);
          if (!SameResult(
                  entry.checker->CheckTrajectory(entry.trajectory(), options),
                  trajectory_expected[k]) ||
              !SameResult(entry.checker->CheckEdge(q1, q2, options),
                          edge_expected[k]) ||
              !SameResult(entry.checker->CheckPath(waypoints[k], options),
                          path_expected[k])) {
            ++mismatches[t];
          }
        }
      }
    });
  }
  for (std::thread& thread : threads) thread.join();
  for (int t = 0; t < kThreads; ++t) EXPECT_EQ(mismatches[t], 0);
}

// The sharing path only ever runs on a workload big enough to hire a helper,
// which the corpus cases above never are. The two tests below are where it gets
// its coverage, TSan's included.

GTEST_TEST(ConcurrencyTest, DeepWorkloadIsThreadCountInvariant) {
  const DeepWorkload& deep = Deep();
  ASSERT_NE(deep.entry, nullptr);
  // Without this floor the test could silently degenerate into measuring a
  // handful of nodes if the corpus or the bisection ever drifted.
  EXPECT_GE(deep.num_nodes, kMinDeepNodes) << "grazing margin " << deep.margin;
  std::cout << "\n[ concurrency ] deep workload: " << deep.entry->name
            << ", margin " << deep.margin << ", " << deep.num_nodes
            << " nodes\n\n";

  const BezierCurve<double> trajectory = deep.entry->trajectory();
  const Result serial = deep.entry->checker->CheckTrajectory(
      trajectory, deep.options(Parallelism::None()));
  for (const int threads : {4, 16}) {
    SCOPED_TRACE("threads " + std::to_string(threads));
    EXPECT_TRUE(SameResult(
        serial, deep.entry->checker->CheckTrajectory(
                    trajectory, deep.options(Parallelism(threads)))));
  }
}

GTEST_TEST(ConcurrencyTest, DeepWorkloadSurvivesConcurrentParallelCalls) {
  // Several caller threads each asking the *same* checker for internal
  // parallelism on a workload big enough to hire: this is the only test that
  // makes concurrent calls contend for the checker's worker threads as well as
  // its context pool, and the case where a lease returning fewer contexts than
  // the pool was warmed for is the normal outcome rather than an edge case.
  const DeepWorkload& deep = Deep();
  ASSERT_NE(deep.entry, nullptr);
  const Result expected = deep.entry->checker->CheckTrajectory(
      deep.entry->trajectory(), deep.options(Parallelism::None()));

  constexpr int kThreads = 4;
  std::vector<int> mismatches(kThreads, 0);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int r = 0; r < 2; ++r) {
        if (!SameResult(
                deep.entry->checker->CheckTrajectory(
                    deep.entry->trajectory(), deep.options(Parallelism(4))),
                expected)) {
          ++mismatches[t];
        }
      }
    });
  }
  for (std::thread& thread : threads) thread.join();
  for (int t = 0; t < kThreads; ++t) EXPECT_EQ(mismatches[t], 0);
}

}  // namespace
}  // namespace test
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
