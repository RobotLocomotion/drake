/// @file
/// T8 — concurrency determinism (test plan T8; performance requirement
/// P7; parallelism and determinism).
///
/// Four claims are pinned here, on the fixed corpus of ten T4-style random
/// cases (a mix of free and violating) that
/// concurrency_test_utilities.h builds, plus the deep workload it derives
/// from that corpus:
///
///   1. The *answer* does not depend on the thread count. Verdict and earliest
///      witness are identical at Parallelism {1, 2, 8, 16} in both search
///      modes, and in kCertifyAll so are `nodes` and `narrowphase_queries` —
///      the parallel driver explores the same tree, only in a different order.
///      (In kFindFirstViolation the branch-and-bound bound arrives at different
///      times, so the *statistics* are explicitly not deterministic; the
///      reported witness still is.)
///   2. Serial mode is bit-deterministic: two runs produce byte-identical
///      findings and statistics.
///   3. The public Check* methods are safe to call concurrently on one checker
///      instance: eight threads hammering one checker get the same answers as
///      running the same calls one after another.
///   4. The deep workload — the only one big enough that the driver actually
///      hires helpers, which no corpus case is — explores the same tree and
///      reports the same findings at every thread count, and keeps doing so
///      when several callers ask for it at once. This is where the sharing
///      path gets its coverage, TSan's included.
///
/// Every case here is an equality, not a wall-clock claim, so this target runs
/// under every build flavor. The two timing claims that used to live here — a
/// deep tree gets faster with threads, a small check does not get slower — are
/// in concurrency_timing_test.cc, which is excluded from the build flavors
/// that make a duration meaningless.
///
/// TSan. This file is the test to run under ThreadSanitizer. Drake's
/// build carries a `tsan` config, so the invocation is:
///
///    bazel test --config=tsan //planning/continuous_collision:concurrency_test
///
/// On recent kernels the default `vm.mmap_rnd_bits` puts mappings outside
/// the range TSan's shadow memory expects and the runtime aborts with
/// "unexpected memory mapping" before main ever runs; running the test
/// binary under `setarch $(uname -m) -R` (or lowering vm.mmap_rnd_bits to
/// 28) is the standard workaround.
///
/// Result on Drake ~v1.45 at the time of writing: clean — no data races
/// reported over repeated runs, so no suppression file is shipped. That was
/// measured against a prebuilt (uninstrumented) Drake, so TSan saw only
/// continuous_collision frames. It sees all of the
/// driver's shared mutable state, though — the work queue, the findings sink,
/// the atomic node counter and bound, and the context pool are all ours — which
/// is exactly the surface the design claims is the only one there is. If a
/// future pin
/// does produce reports rooted entirely in Drake, triage them and park
/// them in a suppression file (TSAN_OPTIONS=suppressions=...); anything
/// rooted in a
/// continuous_collision frame is a real bug.

#include <cstddef>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/parallelism.h"
#include "drake/planning/continuous_collision/test/concurrency_test_utilities.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace test {
namespace {

// ---------------------------------------------------------------------------
// 1. The answer does not depend on the thread count.
// ---------------------------------------------------------------------------

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

GTEST_TEST(ConcurrencyTest, VerdictAndEarliestWitnessAreThreadCountInvariant) {
  for (const SearchMode mode :
       {SearchMode::kCertifyAll, SearchMode::kFindFirstViolation}) {
    for (const auto& entry : Corpus()) {
      const BezierCurve<double> trajectory = entry->trajectory();
      const CertificationResult serial = entry->checker->CheckTrajectory(
          trajectory, BaseOptions(Parallelism::None(), mode));
      for (const int threads : {2, 8, 16}) {
        SCOPED_TRACE(entry->name + ", mode " +
                     (mode == SearchMode::kCertifyAll ? "kCertifyAll"
                                                      : "kFindFirstViolation") +
                     ", threads " + std::to_string(threads));
        const CertificationResult parallel = entry->checker->CheckTrajectory(
            trajectory, BaseOptions(Parallelism(threads), mode));
        EXPECT_EQ(serial.verdict, parallel.verdict);
        EXPECT_TRUE(EarliestWitnessIdentical(serial, parallel));
      }
    }
  }
}

GTEST_TEST(ConcurrencyTest, CertifyAllIsFullyThreadCountInvariant) {
  // In kCertifyAll every node's decision depends only on its own control points
  // and inherited active set, so the *whole* tree — and therefore every
  // statistic and every finding — is thread-count independent, not just the
  // earliest witness.
  for (const auto& entry : Corpus()) {
    const BezierCurve<double> trajectory = entry->trajectory();
    const CertificationResult serial = entry->checker->CheckTrajectory(
        trajectory, BaseOptions(Parallelism::None(), SearchMode::kCertifyAll));
    for (const int threads : {2, 8, 16}) {
      SCOPED_TRACE(entry->name + ", threads " + std::to_string(threads));
      const CertificationResult parallel = entry->checker->CheckTrajectory(
          trajectory,
          BaseOptions(Parallelism(threads), SearchMode::kCertifyAll));
      EXPECT_EQ(serial.stats.nodes, parallel.stats.nodes);
      EXPECT_EQ(serial.stats.narrowphase_queries,
                parallel.stats.narrowphase_queries);
      EXPECT_EQ(serial.stats.sphere_certifications,
                parallel.stats.sphere_certifications);
      EXPECT_EQ(serial.stats.max_depth, parallel.stats.max_depth);
      EXPECT_TRUE(FindingsIdentical(serial.findings, parallel.findings));
    }
  }
}

GTEST_TEST(ConcurrencyTest, FindFirstViolationStatisticsAreAllowedToDiffer) {
  // The complement of the test above, pinned so that a future reader does not
  // "fix" a statistics mismatch that the design explicitly permits: under
  // branch-and-bound the number of nodes a run visits depends on when the
  // atomic bound tightens, which depends on timing. Only the answer is
  // deterministic. (The assertion is therefore on the *witness*, and the
  // statistics are merely reported.)
  int cases_with_differing_stats = 0;
  int examined = 0;
  for (const auto& entry : Corpus()) {
    if (entry->serial_verdict != Verdict::kViolationFound) continue;
    ++examined;
    const BezierCurve<double> trajectory = entry->trajectory();
    const CertificationResult serial = entry->checker->CheckTrajectory(
        trajectory,
        BaseOptions(Parallelism::None(), SearchMode::kFindFirstViolation));
    const CertificationResult parallel = entry->checker->CheckTrajectory(
        trajectory,
        BaseOptions(Parallelism(16), SearchMode::kFindFirstViolation));
    ASSERT_EQ(serial.verdict, parallel.verdict);
    ASSERT_EQ(serial.findings.size(), 1u);
    ASSERT_EQ(parallel.findings.size(), 1u);
    EXPECT_TRUE(EarliestWitnessIdentical(serial, parallel));
    if (serial.stats.nodes != parallel.stats.nodes) {
      ++cases_with_differing_stats;
    }
  }
  // Without this the `continue` above could silently empty the test.
  EXPECT_GE(examined, kMinViolatingCases);
  std::cout << "\n[ T8 ] kFindFirstViolation: node counts differed between 1 "
               "and 16 threads on "
            << cases_with_differing_stats << " of the " << examined
            << " violating cases; the reported witness was identical on all of "
               "them.\n\n";
}

// ---------------------------------------------------------------------------
// 2. Serial mode is bit-deterministic (the performance requirements, P7).
// ---------------------------------------------------------------------------

GTEST_TEST(ConcurrencyTest, SerialModeIsBitDeterministic) {
  for (const auto& entry : Corpus()) {
    for (const SearchMode mode :
         {SearchMode::kCertifyAll, SearchMode::kFindFirstViolation}) {
      SCOPED_TRACE(entry->name);
      const Options options = BaseOptions(Parallelism::None(), mode);
      const BezierCurve<double> trajectory = entry->trajectory();
      const CertificationResult first =
          entry->checker->CheckTrajectory(trajectory, options);
      const CertificationResult second =
          entry->checker->CheckTrajectory(trajectory, options);
      EXPECT_EQ(first.verdict, second.verdict);
      EXPECT_TRUE(FindingsIdentical(first.findings, second.findings));
      EXPECT_EQ(first.stats.nodes, second.stats.nodes);
      EXPECT_EQ(first.stats.narrowphase_queries,
                second.stats.narrowphase_queries);
      EXPECT_EQ(first.stats.sphere_certifications,
                second.stats.sphere_certifications);
      EXPECT_EQ(first.stats.max_depth, second.stats.max_depth);
    }
  }
}

// ---------------------------------------------------------------------------
// 3. Concurrent Check* calls on one checker instance.
// ---------------------------------------------------------------------------

GTEST_TEST(ConcurrencyTest, ConcurrentCallsOnOneCheckerMatchSequential) {
  // Every worker hits the *same* checker object, so they contend for the
  // construction-time context pool; the lease must hand each call its own
  // contexts. Each worker also asks for internal parallelism, so the pool is
  // under pressure from both directions at once.
  const auto& corpus = Corpus();
  const Options options = BaseOptions(Parallelism(2), SearchMode::kCertifyAll);

  std::vector<CertificationResult> sequential;
  for (const auto& entry : corpus) {
    sequential.push_back(
        entry->checker->CheckTrajectory(entry->trajectory(), options));
  }

  constexpr int kThreads = 8;
  constexpr int kRepeats = 3;
  std::vector<std::vector<CertificationResult>> concurrent(kThreads * kRepeats);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int r = 0; r < kRepeats; ++r) {
        std::vector<CertificationResult>& slot = concurrent[t * kRepeats + r];
        for (const auto& entry : corpus) {
          slot.push_back(
              entry->checker->CheckTrajectory(entry->trajectory(), options));
        }
      }
    });
  }
  for (std::thread& thread : threads) thread.join();

  for (int i = 0; i < kThreads * kRepeats; ++i) {
    ASSERT_EQ(concurrent[i].size(), sequential.size());
    for (std::size_t k = 0; k < sequential.size(); ++k) {
      SCOPED_TRACE("worker " + std::to_string(i) + ", case " + corpus[k]->name);
      EXPECT_EQ(concurrent[i][k].verdict, sequential[k].verdict);
      EXPECT_TRUE(
          FindingsIdentical(concurrent[i][k].findings, sequential[k].findings));
      EXPECT_EQ(concurrent[i][k].stats.nodes, sequential[k].stats.nodes);
      EXPECT_EQ(concurrent[i][k].stats.narrowphase_queries,
                sequential[k].stats.narrowphase_queries);
    }
  }
}

GTEST_TEST(ConcurrencyTest, ConcurrentMixedApiCallsAreIndependent) {
  // The same, through the other two public entry points and the const
  // introspection seams, so that a mutable-state regression in any of them
  // shows up here rather than in a user's planner.
  const Case& entry = *Corpus().front();
  const Options options = BaseOptions(Parallelism(2), SearchMode::kCertifyAll);
  const int n = entry.model->plant().num_positions();
  const VectorXd q1 = entry.control_points.col(0);
  const VectorXd q2 = entry.control_points.rightCols(1);
  Eigen::MatrixXd waypoints(n, 3);
  waypoints.col(0) = q1;
  waypoints.col(1) = 0.5 * (q1 + q2);
  waypoints.col(2) = q2;

  const CertificationResult edge_expected =
      entry.checker->CheckEdge(q1, q2, options);
  const CertificationResult path_expected =
      entry.checker->CheckPath(waypoints, options);
  const MotionBoundTable table_expected = entry.checker->ComputeMotionBounds(
      entry.checker->Normalize(entry.trajectory(), options));
  // Snapshot every λ entry, not just the CSR's size: the row layout is fixed by
  // topology and would survive any amount of corruption in the coefficients.
  std::vector<std::vector<std::pair<int, double>>> lambda_expected;
  std::vector<double> slack_expected;
  for (int p = 0; p < table_expected.num_pairs(); ++p) {
    lambda_expected.push_back(table_expected.GetEntries(p));
    slack_expected.push_back(table_expected.carveout_slack(p));
  }

  const auto same_result = [](const CertificationResult& a,
                              const CertificationResult& b) {
    return a.verdict == b.verdict && a.stats.nodes == b.stats.nodes &&
           a.stats.narrowphase_queries == b.stats.narrowphase_queries &&
           a.stats.sphere_certifications == b.stats.sphere_certifications &&
           FindingsIdentical(a.findings, b.findings);
  };

  constexpr int kThreads = 8;
  // gtest assertions are not safe off the main thread, so each worker counts
  // its own mismatches into its own slot and the main thread does the asserting
  // after the join.
  std::vector<int> mismatches(kThreads, 0);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int r = 0; r < 4; ++r) {
        if (!same_result(entry.checker->CheckEdge(q1, q2, options),
                         edge_expected)) {
          ++mismatches[t];
        }
        if (!same_result(entry.checker->CheckPath(waypoints, options),
                         path_expected)) {
          ++mismatches[t];
        }
        const MotionBoundTable table = entry.checker->ComputeMotionBounds(
            entry.checker->Normalize(entry.trajectory(), options));
        if (table.num_pairs() != table_expected.num_pairs()) {
          ++mismatches[t];
          continue;
        }
        for (int p = 0; p < table.num_pairs(); ++p) {
          if (table.GetEntries(p) != lambda_expected[p]) ++mismatches[t];
          // The carve-out residual is part of Δ_p, so it has to be
          // bit-identical across threads too.
          if (table.carveout_slack(p) != slack_expected[p]) ++mismatches[t];
        }
      }
    });
  }
  for (std::thread& thread : threads) thread.join();
  for (int t = 0; t < kThreads; ++t) EXPECT_EQ(mismatches[t], 0);
}

// ---------------------------------------------------------------------------
// 4. The deep workload explores the same tree at every thread count.
// ---------------------------------------------------------------------------
//
// The sharing path only ever runs on a workload big enough to hire a helper,
// which the corpus cases of claims 1-3 never are. These cases are where it
// gets its coverage — including its TSan coverage — and they are equalities,
// so unlike the wall-clock claims in concurrency_timing_test.cc they run
// everywhere.

GTEST_TEST(ConcurrencyTest, DeepWorkloadIsBigEnoughToBeWorthSpreading) {
  // Without this the two tests below could silently degenerate into measuring
  // a handful of nodes if the corpus or the bisection ever drifted.
  const DeepWorkload& deep = Deep();
  ASSERT_NE(deep.entry, nullptr);
  EXPECT_GE(deep.nodes, kMinDeepNodes) << "grazing margin " << deep.margin;
  std::cout << "\n[ T8 ] deep workload: " << deep.entry->name << ", margin "
            << deep.margin << ", " << deep.nodes << " nodes at min_interval "
            << deep.min_interval << "\n\n";
}

GTEST_TEST(ConcurrencyTest, DeepWorkloadIsThreadCountInvariant) {
  // The scaling test below only proves work moved between threads; this proves
  // the *same* work moved. It runs in every build, sanitizers included, and is
  // where the sharing path gets its TSan coverage — the corpus cases of the
  // tests above are too small to ever hire a helper.
  const DeepWorkload& deep = Deep();
  ASSERT_NE(deep.entry, nullptr);
  const BezierCurve<double> trajectory = deep.entry->trajectory();
  const CertificationResult serial = deep.entry->checker->CheckTrajectory(
      trajectory, deep.options(Parallelism::None()));
  for (const int threads : {4, 16}) {
    SCOPED_TRACE("threads " + std::to_string(threads));
    const CertificationResult parallel = deep.entry->checker->CheckTrajectory(
        trajectory, deep.options(Parallelism(threads)));
    EXPECT_EQ(serial.verdict, parallel.verdict);
    EXPECT_EQ(serial.stats.nodes, parallel.stats.nodes);
    EXPECT_EQ(serial.stats.narrowphase_queries,
              parallel.stats.narrowphase_queries);
    EXPECT_EQ(serial.stats.sphere_certifications,
              parallel.stats.sphere_certifications);
    EXPECT_EQ(serial.stats.max_depth, parallel.stats.max_depth);
    EXPECT_TRUE(FindingsIdentical(serial.findings, parallel.findings));
  }
}

GTEST_TEST(ConcurrencyTest, DeepWorkloadSurvivesConcurrentParallelCalls) {
  // Several caller threads each asking the *same* checker for internal
  // parallelism on a workload big enough to hire: this is the only test that
  // makes concurrent calls contend for the checker's worker pool as well as
  // its context pool, and the case where a reservation returning fewer threads
  // than asked for is the normal outcome rather than an edge case.
  const DeepWorkload& deep = Deep();
  ASSERT_NE(deep.entry, nullptr);
  const CertificationResult expected = deep.entry->checker->CheckTrajectory(
      deep.entry->trajectory(), deep.options(Parallelism::None()));

  constexpr int kThreads = 4;
  // gtest assertions are not safe off the main thread, so each worker counts
  // its own mismatches and the main thread asserts after the join.
  std::vector<int> mismatches(kThreads, 0);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int r = 0; r < 2; ++r) {
        const CertificationResult result = deep.entry->checker->CheckTrajectory(
            deep.entry->trajectory(), deep.options(Parallelism(4)));
        if (result.verdict != expected.verdict ||
            result.stats.nodes != expected.stats.nodes ||
            result.stats.narrowphase_queries !=
                expected.stats.narrowphase_queries ||
            result.stats.sphere_certifications !=
                expected.stats.sphere_certifications ||
            !FindingsIdentical(result.findings, expected.findings)) {
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
