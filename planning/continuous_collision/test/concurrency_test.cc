/// @file
/// T8 — concurrency determinism (test plan T8; performance requirement
/// P7; parallelism and determinism).
///
/// Four claims are pinned here. The first three run on a fixed corpus of ten
/// T4-style random cases (a mix of free and violating); the fourth builds one
/// deliberately deep workload out of that corpus:
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
///   4. Per-call parallelism actually distributes work, and never costs
///      anything when there is not enough of it to distribute: a deep tree
///      inside one segment gets measurably faster with threads, and a check
///      too small to pay for workers is no slower at Parallelism::Max() than
///      serially. These are the two regressions the driver rework of
///      certifier_internal.cc fixed, as the benchmark suite's thread-scaling
///      results measured; the deep
///      workload is also where the sharing path gets its TSan coverage, since
///      the corpus cases of claims 1-3 are far too small to hire a helper.
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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/parallelism.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/planning/continuous_collision/continuous_collision_checker.h"
#include "drake/planning/robot_diagram.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::Parallelism;
using drake::geometry::Box;
using drake::geometry::Capsule;
using drake::geometry::Cylinder;
using drake::geometry::HalfSpace;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::planning::RobotDiagram;
using drake::planning::RobotDiagramBuilder;
using drake::trajectories::BezierCurve;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kMargin = 0.005;
/// Ten cases keeps the full 4-thread-count × 2-mode sweep (80 certification
/// runs) plus the concurrent-call test under a second in Release, which is what
/// makes this affordable to run again under TSan (~100× slower).
constexpr int kNumCases = 10;
constexpr int kMinFreeCases = 3;
constexpr int kMinViolatingCases = 3;

CoulombFriction<double> Friction() {
  return CoulombFriction<double>(1.0, 1.0);
}

SpatialInertia<double> Inertia() {
  return SpatialInertia<double>::SolidSphereWithMass(1.0, 0.05);
}

/// A four-link chain of revolute and prismatic joints with primitive geometry,
/// four anchored obstacles and (on odd seeds) a HalfSpace floor, so the corpus
/// exercises the native narrowphase route and the analytic one.
std::unique_ptr<RobotDiagram<double>> MakeWorld(uint64_t seed) {
  std::mt19937_64 rng(seed);
  const auto uniform = [&rng](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
  };
  // Every helper below sequences its draws through named locals: the order in
  // which a compiler evaluates sibling constructor or operator arguments is
  // unspecified, so drawing inline would make the corpus toolchain-dependent
  // and could silently shift the free/violating balance this file relies on.
  const auto vector3 = [&uniform](double lo, double hi) {
    const double x = uniform(lo, hi);
    const double y = uniform(lo, hi);
    const double z = uniform(lo, hi);
    return Vector3d(x, y, z);
  };
  const auto direction = [&vector3]() {
    Vector3d v;
    do {
      v = vector3(-1, 1);
    } while (v.norm() < 1e-3 || v.norm() > 1.0);
    return v.normalized();
  };
  const auto offset = [&direction, &uniform](double lo, double hi) {
    const Vector3d unit = direction();
    const double length = uniform(lo, hi);
    return Vector3d(unit * length);
  };
  const auto pose = [&vector3, &offset](double lo, double hi) {
    const Vector3d rpy = vector3(-3, 3);
    const Vector3d p = offset(lo, hi);
    return RigidTransformd(RollPitchYawd(rpy), p);
  };

  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  std::vector<const RigidBody<double>*> links;
  for (int i = 0; i < 4; ++i) {
    const std::string name = "link" + std::to_string(i);
    const RigidBody<double>& body = plant.AddRigidBody(name, Inertia());
    const RigidBody<double>& parent =
        (i == 0) ? plant.world_body() : *links.back();
    const Vector3d rpy_PF = vector3(-0.5, 0.5);
    const RigidTransformd X_PF(RollPitchYawd(rpy_PF), offset(0.22, 0.32));
    const Vector3d axis = direction();
    if (i == 2) {
      plant.AddJoint<PrismaticJoint>("j" + std::to_string(i), parent, X_PF,
                                     body, RigidTransformd(), axis);
    } else {
      plant.AddJoint<RevoluteJoint>("j" + std::to_string(i), parent, X_PF, body,
                                    RigidTransformd(), axis);
    }
    const RigidTransformd X_LG(offset(0.10, 0.16));
    if (i % 2 == 0) {
      const double radius = uniform(0.02, 0.04);
      const double length = uniform(0.05, 0.10);
      plant.RegisterCollisionGeometry(body, X_LG, Capsule(radius, length),
                                      name + "_geom", Friction());
    } else {
      const Vector3d size = vector3(0.04, 0.09);
      plant.RegisterCollisionGeometry(body, X_LG,
                                      Box(size.x(), size.y(), size.z()),
                                      name + "_geom", Friction());
    }
    links.push_back(&body);
  }
  for (int i = 0; i < 4; ++i) {
    const std::string name = "obstacle" + std::to_string(i);
    const RigidBody<double>& body = plant.AddRigidBody(name, Inertia());
    plant.WeldFrames(plant.world_frame(), body.body_frame(), pose(0.30, 0.75));
    if (i % 3 == 0) {
      plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                      Sphere(uniform(0.05, 0.12)),
                                      name + "_geom", Friction());
    } else if (i % 3 == 1) {
      const Vector3d size = vector3(0.08, 0.20);
      plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                      Box(size.x(), size.y(), size.z()),
                                      name + "_geom", Friction());
    } else {
      const double radius = uniform(0.04, 0.09);
      const double length = uniform(0.08, 0.18);
      plant.RegisterCollisionGeometry(body, RigidTransformd(),
                                      Cylinder(radius, length), name + "_geom",
                                      Friction());
    }
  }
  if (seed % 2 == 1) {
    const RigidBody<double>& floor = plant.AddRigidBody("floor", Inertia());
    plant.WeldFrames(plant.world_frame(), floor.body_frame(),
                     RigidTransformd(Vector3d(0.0, 0.0, -0.5)));
    plant.RegisterCollisionGeometry(floor, RigidTransformd(), HalfSpace(),
                                    "floor_geom", Friction());
  }
  return builder.Build();
}

/// A quintic Bézier with random control points, so the corpus has real curved
/// trajectories rather than straight edges.
Eigen::MatrixXd MakeControlPoints(uint64_t seed, int num_positions) {
  std::mt19937_64 rng(seed ^ 0xa5a5'5a5a'0f0f'f0f0ull);
  std::uniform_real_distribution<double> value(-1.4, 1.4);
  Eigen::MatrixXd points(num_positions, 6);
  for (int j = 0; j < 6; ++j) {
    for (int i = 0; i < num_positions; ++i) points(i, j) = value(rng);
  }
  return points;
}

Options BaseOptions(Parallelism parallelism, SearchMode mode) {
  Options options;
  options.margin = kMargin;
  options.parallelism = parallelism;
  options.mode = mode;
  // Bounded cost per run: the whole sweep is executed 8 times per case.
  options.min_interval = 1e-6;
  return options;
}

struct Case {
  std::string name;
  std::shared_ptr<const RobotDiagram<double>> model;
  std::unique_ptr<ContinuousCollisionChecker> checker;
  Eigen::MatrixXd control_points;
  Verdict serial_verdict{};

  BezierCurve<double> trajectory() const {
    return BezierCurve<double>(0.0, 1.0, control_points);
  }
};

/// Ten cases with at least three free and three violating, taken from the
/// lowest seeds that supply them (deterministic, no hard-coded lucky numbers).
///
/// The vector is deliberately allocated and never freed: it owns RobotDiagrams
/// and checkers whose destruction would otherwise race Drake's own static
/// teardown. (Expect LSan to report it if an asan preset is ever added next to
/// the tsan one.)
const std::vector<std::unique_ptr<Case>>& Corpus() {
  static const std::vector<std::unique_ptr<Case>>* corpus = [] {
    auto* cases = new std::vector<std::unique_ptr<Case>>();
    int free_count = 0;
    int violating_count = 0;
    for (uint64_t seed = 1; seed <= 200; ++seed) {
      if (static_cast<int>(cases->size()) >= kNumCases) break;
      auto entry = std::make_unique<Case>();
      entry->name = "seed_" + std::to_string(seed);
      entry->model = MakeWorld(seed);
      ContinuousCollisionChecker::Params params;
      params.model = entry->model;
      params.default_options =
          BaseOptions(Parallelism::None(), SearchMode::kCertifyAll);
      entry->checker = std::make_unique<ContinuousCollisionChecker>(params);
      entry->control_points =
          MakeControlPoints(seed, entry->model->plant().num_positions());
      const CertificationResult result = entry->checker->CheckTrajectory(
          entry->trajectory(),
          BaseOptions(Parallelism::None(), SearchMode::kCertifyAll));
      entry->serial_verdict = result.verdict;
      // Keep the corpus balanced: stop taking more of whichever kind is
      // already well represented.
      const bool is_free = result.verdict == Verdict::kCertifiedFree;
      const bool is_violating = result.verdict == Verdict::kViolationFound;
      if (!is_free && !is_violating) continue;
      if (is_free && free_count >= kNumCases - kMinViolatingCases) continue;
      if (is_violating && violating_count >= kNumCases - kMinFreeCases) {
        continue;
      }
      (is_free ? free_count : violating_count) += 1;
      cases->push_back(std::move(entry));
    }
    return cases;
  }();
  return *corpus;
}

/// Bit-for-bit equality of two findings. Nothing here is a tolerance: two runs
/// of the same deterministic computation either agree exactly or the claim of
/// determinism is false.
::testing::AssertionResult FindingsIdentical(const std::vector<Finding>& a,
                                             const std::vector<Finding>& b) {
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure()
           << "finding counts differ: " << a.size() << " vs " << b.size();
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].time != b[i].time) {
      return ::testing::AssertionFailure()
             << "finding " << i << " time " << a[i].time << " vs " << b[i].time;
    }
    if (a[i].q.size() != b[i].q.size() ||
        !(a[i].q.array() == b[i].q.array()).all()) {
      return ::testing::AssertionFailure()
             << "finding " << i << " witness configuration differs";
    }
    if (a[i].pair.a != b[i].pair.a || a[i].pair.b != b[i].pair.b) {
      return ::testing::AssertionFailure()
             << "finding " << i << " pair differs";
    }
    if (a[i].distance != b[i].distance ||
        a[i].motion_bound != b[i].motion_bound ||
        a[i].definite != b[i].definite) {
      return ::testing::AssertionFailure()
             << "finding " << i << " payload differs";
    }
    if (a[i].nearest_a_W.has_value() != b[i].nearest_a_W.has_value() ||
        (a[i].nearest_a_W.has_value() &&
         *a[i].nearest_a_W != *b[i].nearest_a_W)) {
      return ::testing::AssertionFailure()
             << "finding " << i << " witness point A differs";
    }
    if (a[i].nearest_b_W.has_value() != b[i].nearest_b_W.has_value() ||
        (a[i].nearest_b_W.has_value() &&
         *a[i].nearest_b_W != *b[i].nearest_b_W)) {
      return ::testing::AssertionFailure()
             << "finding " << i << " witness point B differs";
    }
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult EarliestWitnessIdentical(
    const CertificationResult& a, const CertificationResult& b) {
  if (a.findings.empty() != b.findings.empty()) {
    return ::testing::AssertionFailure()
           << "one run reported findings and the other did not";
  }
  if (a.findings.empty()) return ::testing::AssertionSuccess();
  return FindingsIdentical({a.findings.front()}, {b.findings.front()});
}

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
// 4. Per-call parallel scaling.
// ---------------------------------------------------------------------------
//
// These pin the two properties the driver rework of certifier_internal.cc
// exists for, and that the benchmark suite's thread-scaling results measured
// the old driver failing:
//
//   a) a deep tree inside a single segment actually spreads over the workers
//      (the old depth-seeded driver got 0.98× at 16 threads on 12 570 nodes,
//      because one fixed seed held essentially the whole tree);
//   b) a check too small to pay for workers never loses by being asked for
//      them — which matters because Parallelism::Max() is the *default* value
//      of Options::parallelism.
//
// Both are timing claims, so both are written to survive a loaded machine: a
// ratio with a wide margin, best-of-three, and a skip when the hardware or the
// build cannot support the claim at all. They are not benchmarks — the numbers
// live in benchmark/results/ — they are regression detectors, and they should
// only ever fire on a driver that has stopped distributing work.

/// True when the build cannot support a meaningful wall-clock claim: a
/// sanitizer build serializes and inflates everything, an unoptimized build
/// changes the ratios, and fewer than eight hardware threads means there is no
/// parallelism to measure.
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
  return std::thread::hardware_concurrency() < 8;
#endif
}

template <typename F>
double BestOfThreeSeconds(F&& body) {
  body();  // Warm up: first-touch page faults, the worker pool's threads.
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

/// The bisection's node budget below doubles as the deep workload's size: the
/// margin it converges to is the largest one still certifiable inside this
/// budget, so the tree it produces has just under this many nodes. Large
/// enough that a run takes tens of milliseconds (a wall-clock ratio then means
/// something) and that no fixed seeding depth could ever have covered it;
/// small enough that the ~40 probes that find it, and the timed repetitions
/// that use it, stay cheap — under a sanitizer too.
constexpr uint64_t kProbeBudget = 6000;
constexpr uint64_t kMinDeepNodes = 3000;

/// A corpus case run at a margin just below its own swept clearance, which is
/// what makes the subdivision tree deep and *narrow* (the soundness argument):
/// certifying a node needs φ̂ − τ − Δ > m, so as the threshold m approaches the
/// trajectory's closest approach the motion bound Δ has to be driven to nothing
/// there and nowhere else. The result is thousands of nodes concentrated in a
/// tiny sub-interval of one segment — exactly the shape a depth-seeded work
/// queue cannot split, and the shape the benchmark suite's thread-scaling
/// results measured the old driver getting 0.98× on.
///
/// That margin is found by bisection rather than hard-coded, so the workload
/// survives any change to the random worlds, the bounds, or Drake: the largest
/// margin still certifiable within kProbeBudget nodes is by construction the
/// one that costs about kProbeBudget nodes.
struct DeepWorkload {
  const Case* entry{};
  double margin{0.0};
  double min_interval{1e-8};
  uint64_t nodes{0};

  Options options(Parallelism parallelism) const {
    Options options = BaseOptions(parallelism, SearchMode::kCertifyAll);
    options.margin = margin;
    options.min_interval = min_interval;
    return options;
  }
};

const DeepWorkload& Deep() {
  static const DeepWorkload* workload = []() {
    auto* deep = new DeepWorkload();
    for (const auto& entry : Corpus()) {
      if (entry->serial_verdict != Verdict::kCertifiedFree) continue;
      deep->entry = entry.get();
      break;
    }
    if (deep->entry == nullptr) return deep;

    const auto certifiable_within_budget = [&](double margin) {
      Options options = deep->options(Parallelism::None());
      options.margin = margin;
      options.max_nodes = kProbeBudget;
      return deep->entry->checker
                 ->CheckTrajectory(deep->entry->trajectory(), options)
                 .verdict == Verdict::kCertifiedFree;
    };
    double certifiable = 0.0;
    double grazing = kMargin;
    for (int i = 0; i < 12 && certifiable_within_budget(grazing); ++i) {
      certifiable = grazing;
      grazing *= 2.0;
    }
    for (int i = 0; i < 30; ++i) {
      const double mid = 0.5 * (certifiable + grazing);
      (certifiable_within_budget(mid) ? certifiable : grazing) = mid;
    }
    deep->margin = certifiable;
    deep->nodes = deep->entry->checker
                      ->CheckTrajectory(deep->entry->trajectory(),
                                        deep->options(Parallelism::None()))
                      .stats.nodes;
    return deep;
  }();
  return *workload;
}

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
  std::cout << "\n[ T8 ] deep workload: serial " << 1e3 * serial
            << " ms, Parallelism(8) " << 1e3 * parallel << " ms ("
            << serial / parallel << "x)\n\n";
  // Eight threads measure ~6x on the benchmark machine; 1.43x is the bound
  // that separates "the driver distributes deep work" from the old driver's
  // 0.98x without being a performance assertion in disguise.
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
  std::cout << "\n[ T8 ] small check: serial " << 1e3 * serial
            << " ms, Parallelism::Max() " << 1e3 * parallel << " ms ("
            << serial / parallel << "x)\n\n";
  // Parity is what the driver actually delivers (it never hires for a check
  // this small, so the two paths run the same code); the 1.5x bound leaves
  // room for scheduler noise on a loaded machine without letting a return of
  // the old 2.6x slowdown through.
  EXPECT_LT(parallel, 1.5 * serial);
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
