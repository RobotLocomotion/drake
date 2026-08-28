// End-to-end tests of the certifier core and the public facade on a focused,
// hand-built corpus. The large randomized corpus lives in
// test/soundness_fuzz_test.cc, the certificate mutation sweep in
// test/certificate_test.cc, the thread-count sweep in test/concurrency_test.cc
// and the API throw conditions in test/api_test.cc; none is duplicated here.
//
// Every world is built programmatically, every trajectory is fixed, and every
// cross-check is dense sampling of the *same* path the checker certified, so
// the suite is deterministic and fast.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/planning/continuous_collision/test/test_utilities.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using test::BezierCurve;
using test::Box;
using test::DistanceAtFinding;
using test::Friction;
using test::HalfSpace;
using test::Inertia;
using test::MakeChecker;
using test::MultibodyPlant;
using test::Parallelism;
using test::PrismaticJoint;
using test::QueryObject;
using test::RevoluteJoint;
using test::RigidBody;
using test::RigidTransformd;
using test::RobotDiagram;
using test::RobotDiagramBuilder;
using test::Sphere;

constexpr double kMargin = 0.01;

// A planar 3-dof arm (revolute, revolute, prismatic) in the z = 0 plane:
// clang-format off
//   world --j1(Rz)--> link1 [box, x ∈ 0 .. 0.40]
//                       --j2(Rz @ x=0.40)--> link2 [box, x ∈ 0 .. 0.30]
//                            --j3(Px @ x=0.30)--> tool [sphere r = 0.05]
// clang-format on
// so q = (θ1, θ2, d) and the tool centre sits at radius ≈ 0.70 + d when the
// arm is straight. Obstacles are welded to the world.
void AddArm(MultibodyPlant<double>* plant) {
  const RigidBody<double>& link1 = plant->AddRigidBody("link1", Inertia());
  const RigidBody<double>& link2 = plant->AddRigidBody("link2", Inertia());
  const RigidBody<double>& tool = plant->AddRigidBody("tool", Inertia());

  plant->AddJoint<RevoluteJoint>("j1", plant->world_body(), {}, link1, {},
                                 Vector3d::UnitZ());
  plant->AddJoint<RevoluteJoint>("j2", link1,
                                 RigidTransformd(Vector3d(0.40, 0.0, 0.0)),
                                 link2, {}, Vector3d::UnitZ());
  plant->AddJoint<PrismaticJoint>("j3", link2,
                                  RigidTransformd(Vector3d(0.30, 0.0, 0.0)),
                                  tool, {}, Vector3d::UnitX());

  plant->RegisterCollisionGeometry(
      link1, RigidTransformd(Vector3d(0.20, 0.0, 0.0)), Box(0.40, 0.06, 0.06),
      "link1_geom", Friction());
  plant->RegisterCollisionGeometry(
      link2, RigidTransformd(Vector3d(0.15, 0.0, 0.0)), Box(0.30, 0.06, 0.06),
      "link2_geom", Friction());
  plant->RegisterCollisionGeometry(tool, RigidTransformd(), Sphere(0.05),
                                   "tool_geom", Friction());
}

void AddWeldedSphere(MultibodyPlant<double>* plant, const std::string& name,
                     const Vector3d& p_W, double radius) {
  const RigidBody<double>& body = plant->AddRigidBody(name, Inertia());
  plant->WeldFrames(plant->world_frame(), body.body_frame(),
                    RigidTransformd(p_W));
  plant->RegisterCollisionGeometry(body, RigidTransformd(), Sphere(radius),
                                   name + "_geom", Friction());
}

// The main world: the arm, two round obstacles at different sweep angles, a
// ground halfspace (which exercises the analytic distance route and the
// "skip the sphere prefilter" path) and a far ceiling box.
std::shared_ptr<const RobotDiagram<double>> MakeArmWorld() {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  AddArm(&plant);
  // Angle ≈ 1.571 rad from the arm's home direction, radius 0.75.
  AddWeldedSphere(&plant, "post", Vector3d(0.0, 0.75, 0.0), 0.10);
  // Angle ≈ 2.575 rad, radius 0.65.
  AddWeldedSphere(&plant, "pillar", Vector3d(-0.55, 0.35, 0.0), 0.08);

  const RigidBody<double>& ground = plant.AddRigidBody("ground", Inertia());
  plant.WeldFrames(plant.world_frame(), ground.body_frame(),
                   RigidTransformd(Vector3d(0.0, 0.0, -0.50)));
  plant.RegisterCollisionGeometry(ground, RigidTransformd(), HalfSpace(),
                                  "ground_geom", Friction());

  const RigidBody<double>& ceiling = plant.AddRigidBody("ceiling", Inertia());
  plant.WeldFrames(plant.world_frame(), ceiling.body_frame(),
                   RigidTransformd(Vector3d(0.0, 0.0, 0.90)));
  plant.RegisterCollisionGeometry(ceiling, RigidTransformd(),
                                  Box(2.0, 2.0, 0.20), "ceiling_geom",
                                  Friction());
  return builder.Build();
}

Options SerialOptions() {
  Options options;
  options.margin = kMargin;
  options.parallelism = Parallelism::None();
  return options;
}

// A Bézier from `start` to `end` with linearly spaced control points (so the
// curve is the straight segment, traversed with a nontrivial parametrization)
// over [t0, t1].
BezierCurve<double> MakeBezier(const VectorXd& start, const VectorXd& end,
                               int order, double t0 = 0.0, double t1 = 1.0) {
  Eigen::MatrixXd control_points(start.size(), order + 1);
  for (int j = 0; j <= order; ++j) {
    const double u = static_cast<double>(j) / order;
    control_points.col(j) = (1.0 - u) * start + u * end;
  }
  return BezierCurve<double>(t0, t1, control_points);
}

VectorXd MakeQ(double theta1, double theta2, double d) {
  VectorXd q(3);
  q << theta1, theta2, d;
  return q;
}

// Result of the dense-sampling cross-check.
struct SampledClearance {
  double min_clearance{std::numeric_limits<double>::infinity()};
  // Time of the first sample whose clearance drops below `threshold`, or NaN.
  double first_crossing{std::numeric_limits<double>::quiet_NaN()};
};

// Densely samples `path` and evaluates every unfiltered pair discretely. This
// is the independent check the certifier's continuum claim is measured against;
// it reuses the distance oracle (tested on its own in
// test/distance_oracle_test.cc) so that halfspace pairs are handled the same
// way.
SampledClearance SampleClearance(const ContinuousCollisionChecker& checker,
                                 const PiecewiseBezierPath& path,
                                 int samples_per_segment, double threshold) {
  const RobotDiagram<double>& model = checker.model();
  auto root = model.CreateDefaultContext();
  auto& plant_context = model.plant().GetMyMutableContextFromRoot(root.get());
  const auto& scene_graph = model.scene_graph();
  SampledClearance result;
  for (int k = 0; k < static_cast<int>(path.segments().size()); ++k) {
    const BezierSegment& segment = path.segments()[k];
    for (int i = 0; i <= samples_per_segment; ++i) {
      const double s = static_cast<double>(i) / samples_per_segment;
      const VectorXd q = path.EvaluateSegment(k, s);
      model.plant().SetPositions(&plant_context, q);
      const auto& query_object =
          scene_graph.get_query_output_port().Eval<QueryObject<double>>(
              scene_graph.GetMyContextFromRoot(*root));
      for (const PairRecord& pair : checker.pairs()) {
        const double phi =
            checker.distance_oracle().SignedDistance(query_object, pair);
        result.min_clearance = std::min(result.min_clearance, phi);
        if (phi < threshold && std::isnan(result.first_crossing)) {
          result.first_crossing =
              segment.t_start + s * (segment.t_end - segment.t_start);
        }
      }
    }
  }
  return result;
}

// ---------------------------------------------------------------------------
// 1. Free trajectories are certified, and dense sampling agrees.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, FreeTrajectoryCertified) {
  const auto checker = MakeChecker(MakeArmWorld(), SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10), 3);

  const CertificationResult result = checker.CheckTrajectory(trajectory);
  EXPECT_EQ(result.verdict, Verdict::kCertifiedFree);
  EXPECT_TRUE(result.findings.empty());
  EXPECT_GT(result.stats.nodes, 0u);
  EXPECT_GT(result.stats.sphere_certifications, 0u);
  EXPECT_FALSE(result.certificate.has_value());

  // Independent cross-check: 10^4 dense samples must all clear the margin.
  const PiecewiseBezierPath path = checker.Normalize(trajectory);
  const SampledClearance sampled =
      SampleClearance(checker, path, 10000, kMargin);
  EXPECT_GT(sampled.min_clearance, kMargin);
  EXPECT_TRUE(std::isnan(sampled.first_crossing));

  // The same statement through the other two entry points.
  Eigen::MatrixXd waypoints(3, 3);
  waypoints.col(0) = MakeQ(0.0, 0.0, 0.0);
  waypoints.col(1) = MakeQ(0.4, -0.2, 0.05);
  waypoints.col(2) = MakeQ(0.8, -0.4, 0.10);
  EXPECT_EQ(checker.CheckPath(waypoints).verdict, Verdict::kCertifiedFree);
  EXPECT_EQ(
      checker.CheckEdge(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10)).verdict,
      Verdict::kCertifiedFree);
}

GTEST_TEST(CertifierTest, NarrowGapCertifiedBySubdivision) {
  // A genuinely free squeeze: the tool slides between two spheres that leave
  // only 5 mm of clearance over the margin (0.115 − 0.05 − 0.05 = 0.015 =
  // kMargin + 0.005), so the certificate is real but has to be earned by
  // subdividing.
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  AddArm(&plant);
  AddWeldedSphere(&plant, "gap_left", Vector3d(0.80, 0.115, 0.0), 0.05);
  AddWeldedSphere(&plant, "gap_right", Vector3d(0.80, -0.115, 0.0), 0.05);
  const auto checker = MakeChecker(builder.Build(), SerialOptions());
  // Only the prismatic coordinate moves: the tool slides through the gap.
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.0, 0.0, 0.20), 1);

  Options options = SerialOptions();
  options.emit_certificate = true;
  const CertificationResult result =
      checker.CheckTrajectory(trajectory, options);
  EXPECT_EQ(result.verdict, Verdict::kCertifiedFree);
  EXPECT_TRUE(result.findings.empty());
  // A 5 mm gap over the margin against ~0.2 m of travel cannot be certified at
  // the root: the motion bound has to be tightened by subdivision.
  EXPECT_GE(result.stats.max_depth, 4);

  const PiecewiseBezierPath path = checker.Normalize(trajectory);
  const SampledClearance sampled =
      SampleClearance(checker, path, 10000, kMargin);
  EXPECT_GT(sampled.min_clearance, kMargin);
  EXPECT_LT(sampled.min_clearance, kMargin + 0.01);
  ASSERT_TRUE(result.certificate.has_value());
  EXPECT_TRUE(VerifyCertificate(checker, path, *result.certificate));
}

// ---------------------------------------------------------------------------
// 2. A sweeping trajectory that hits an obstacle.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, ViolationFoundWithExactWitness) {
  const auto checker = MakeChecker(MakeArmWorld(), SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(2.0, 0.0, 0.0), 1);

  const CertificationResult result = checker.CheckTrajectory(trajectory);
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  ASSERT_FALSE(result.findings.empty());
  const Finding& finding = result.findings.front();
  EXPECT_TRUE(finding.definite);
  EXPECT_TRUE(finding.nearest_a_W.has_value());
  EXPECT_TRUE(finding.nearest_b_W.has_value());

  // The witness is exactly on the trajectory, so re-evaluating the path at the
  // reported time must reproduce it; and re-querying the distance from a fresh
  // context must confirm the violation.
  const PiecewiseBezierPath path = checker.Normalize(trajectory);
  EXPECT_LT((path.Value(finding.time) - finding.q).cwiseAbs().maxCoeff(), 1e-9);
  const double phi = DistanceAtFinding(checker, finding);
  EXPECT_LT(phi, kMargin);
  EXPECT_NEAR(phi, finding.distance, 1e-12);
}

GTEST_TEST(CertifierTest, FindFirstReturnsEarliestWitness) {
  const auto checker = MakeChecker(MakeArmWorld(), SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(2.0, 0.0, 0.0), 1);

  Options options = SerialOptions();
  options.mode = SearchMode::kFindFirstViolation;
  const CertificationResult result =
      checker.CheckTrajectory(trajectory, options);
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  ASSERT_EQ(result.findings.size(), 1u);

  const PiecewiseBezierPath path = checker.Normalize(trajectory);
  const SampledClearance sampled =
      SampleClearance(checker, path, 10000, kMargin);
  ASSERT_FALSE(std::isnan(sampled.first_crossing));
  // The branch-and-bound recursion drives the reported witness to the earliest
  // violating time, which dense sampling brackets from above.
  EXPECT_NEAR(result.findings.front().time, sampled.first_crossing, 5e-3);
  EXPECT_LE(result.findings.front().time, sampled.first_crossing + 1e-9);
}

GTEST_TEST(CertifierTest, CertifyAllReportsEveryViolation) {
  const auto checker = MakeChecker(MakeArmWorld(), SerialOptions());
  // Sweeping θ1 from 0 to 3 rad passes the post (≈1.57 rad) and then the
  // pillar (≈2.58 rad): two disjoint violating regions, different pairs.
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(3.0, 0.0, 0.0), 1);

  const CertificationResult result = checker.CheckTrajectory(trajectory);
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  ASSERT_GE(result.findings.size(), 2u);
  int definite = 0;
  for (std::size_t i = 0; i < result.findings.size(); ++i) {
    if (i > 0) {
      EXPECT_LE(result.findings[i - 1].time, result.findings[i].time)
          << "findings must be earliest-first";
    }
    if (result.findings[i].definite) {
      ++definite;
      EXPECT_LT(DistanceAtFinding(checker, result.findings[i]), kMargin);
    }
  }
  EXPECT_GE(definite, 2);
}

GTEST_TEST(CertifierTest, GrazingTangencyIsInconclusive) {
  // A world built for exact tangency: with θ1 = θ2 = 0 held constant the tool
  // centre slides along +x through (0.80, 0, 0), where the "graze" sphere sits
  // at distance 0.11, which is exactly r_tool + r_graze + kMargin.
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  AddArm(&plant);
  AddWeldedSphere(&plant, "graze", Vector3d(0.80, 0.11, 0.0), 0.05);
  const auto checker = MakeChecker(builder.Build(), SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.0, 0.0, 0.20), 1);

  Options options = SerialOptions();
  // A coarser floor keeps the cost of the tangency cascade bounded; the
  // verdict is what matters here, not the depth.
  options.min_interval = 1e-4;
  const CertificationResult result =
      checker.CheckTrajectory(trajectory, options);

  EXPECT_EQ(result.verdict, Verdict::kInconclusive);
  ASSERT_FALSE(result.findings.empty());
  const Finding& finding = result.findings.front();
  EXPECT_FALSE(finding.definite);
  // The near-witness sits within a hair of the threshold.
  EXPECT_NEAR(finding.distance, kMargin, 1e-3);
  EXPECT_NEAR(DistanceAtFinding(checker, finding), finding.distance, 1e-12);

  // Dense sampling confirms the tangency: the minimum clearance touches the
  // margin but (up to sampling) never dips meaningfully below it.
  const PiecewiseBezierPath path = checker.Normalize(trajectory);
  const SampledClearance sampled =
      SampleClearance(checker, path, 10000, kMargin);
  EXPECT_NEAR(sampled.min_clearance, kMargin, 1e-6);
}

// ---------------------------------------------------------------------------
// 3. Static pairs (J(p) = ∅) are resolved once and certified globally.
// ---------------------------------------------------------------------------

// MultibodyPlant::Finalize() already filters every pair *within* a welded
// subgraph, so two anchored obstacles never even reach the checker as a
// candidate pair. The reachable source of J(p) = ∅ is the constant-coordinate
// carve-out: a coordinate that no control point of the trajectory moves is
// removed from every J(p), and pairs left with an empty set are resolved once
// at q(t0).
GTEST_TEST(CertifierTest, StaticPairsResolvedOnce) {
  const auto checker = MakeChecker(MakeArmWorld(), SerialOptions());
  // Only the prismatic coordinate moves: θ1 and θ2 are constant, so every pair
  // whose relative pose depends only on them becomes static.
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.3, -0.2, 0.0), MakeQ(0.3, -0.2, 0.15), 2);

  const PiecewiseBezierPath path = checker.Normalize(trajectory);
  const MotionBoundTable table = checker.ComputeMotionBounds(path);

  int num_static = 0;
  int num_moving = 0;
  for (int p = 0; p < table.num_pairs(); ++p) {
    (table.pair_is_static(p) ? num_static : num_moving) += 1;
  }
  EXPECT_GT(num_static, 0) << "the constant-coordinate carve-out should have "
                              "made the link1/link2 pairs static";
  EXPECT_GT(num_moving, 0);

  Options options = SerialOptions();
  options.emit_certificate = true;
  const CertificationResult result =
      checker.CheckTrajectory(trajectory, options);
  ASSERT_EQ(result.verdict, Verdict::kCertifiedFree);
  ASSERT_TRUE(result.certificate.has_value());

  // A static pair is certified exactly once: one full-segment record per
  // segment, all sharing the single representative configuration q(t0). It
  // never appears in a node record.
  const int num_segments = static_cast<int>(path.segments().size());
  std::vector<int> records_per_pair(table.num_pairs(), 0);
  for (const CertificateRecord& record : result.certificate->records) {
    ++records_per_pair[record.pair_index];
    if (table.pair_is_static(record.pair_index)) {
      EXPECT_EQ(record.s_start, 0.0);
      EXPECT_EQ(record.s_end, 1.0);
      EXPECT_EQ(record.motion_bound, 0.0);
      EXPECT_LT(
          (record.qc - path.EvaluateSegment(0, 0.0)).cwiseAbs().maxCoeff(),
          1e-15);
    }
  }
  for (int p = 0; p < table.num_pairs(); ++p) {
    if (table.pair_is_static(p)) {
      EXPECT_EQ(records_per_pair[p], num_segments)
          << "static pair " << p << " was resolved more than once";
    }
  }
  EXPECT_TRUE(VerifyCertificate(checker, path, *result.certificate));

  // A static record must be measured at the path's own start configuration:
  // "static" is relative to the carve-out, so a record re-based onto an
  // off-path configuration would measure a different pair pose entirely. Both
  // directions: rotating θ1 toward the obstacles reduces the clearance the
  // replay measures, while rotating away *increases* it, and only the "static
  // records are pinned to q(t0)" check catches that second case.
  for (const double delta : {1.5, -1.5}) {
    Certificate certificate = *result.certificate;
    int tampered = 0;
    for (CertificateRecord& record : certificate.records) {
      if (table.pair_is_static(record.pair_index)) {
        record.qc[0] += delta;
        ++tampered;
        break;
      }
    }
    ASSERT_EQ(tampered, 1);
    EXPECT_FALSE(VerifyCertificate(checker, path, certificate))
        << "delta = " << delta;
  }
}

// ---------------------------------------------------------------------------
// 4. Padding reaches the effective threshold, and the env/self split is the
//    documented one (self = both bodies move relative to the world).
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, PaddingSemantics) {
  const auto model = MakeArmWorld();
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10), 3);
  const auto is_arm_self_pair = [&model](const Finding& finding) {
    const auto& plant = model->plant();
    const std::string a = plant.get_body(finding.pair.body_a).name();
    const std::string b = plant.get_body(finding.pair.body_b).name();
    return (a == "link1" || a == "link2" || a == "tool") &&
           (b == "link1" || b == "link2" || b == "tool");
  };

  // The one robot-vs-robot pair (link1, tool) keeps ≈ 0.27 m of clearance on
  // this trajectory, so 0.4 m of *self* padding must break it, and nothing
  // else: every other pair has an anchored side and takes the (zero)
  // environment padding. Mirrored, 0.5 m of environment padding reaches the
  // arm-vs-obstacle pairs (the ground halfspace is 0.45 m away) and leaves the
  // self pair alone.
  for (const bool self : {true, false}) {
    SCOPED_TRACE(self ? "self padding" : "env padding");
    PaddingSpec padding;
    (self ? padding.self_padding : padding.env_padding) = self ? 0.40 : 0.50;
    const auto checker = MakeChecker(model, SerialOptions(), padding);
    const CertificationResult result = checker.CheckTrajectory(trajectory);
    ASSERT_EQ(result.verdict, Verdict::kViolationFound);
    for (const Finding& finding : result.findings) {
      EXPECT_EQ(is_arm_self_pair(finding), self)
          << "padding must apply to exactly one class of pair";
    }
  }

  // A per-body-pair matrix overrides the scalars ...
  PaddingSpec overridden;
  overridden.env_padding = 0.50;
  overridden.per_body_pair = Eigen::MatrixXd::Zero(model->plant().num_bodies(),
                                                   model->plant().num_bodies());
  EXPECT_EQ(MakeChecker(model, SerialOptions(), overridden)
                .CheckTrajectory(trajectory)
                .verdict,
            Verdict::kCertifiedFree);

  // ... and a mis-sized matrix is a clear throw.
  PaddingSpec mis_sized;
  mis_sized.per_body_pair = Eigen::MatrixXd::Zero(2, 2);
  EXPECT_THROW(MakeChecker(model, SerialOptions(), mis_sized), std::exception);
}

// ---------------------------------------------------------------------------
// 5. Retiming invariance: the certificate is a property of the path.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, RetimingInvariance) {
  const auto checker = MakeChecker(MakeArmWorld(), SerialOptions());
  const VectorXd start = MakeQ(0.0, 0.0, 0.0);
  const VectorXd end = MakeQ(0.8, -0.4, 0.10);

  Options options = SerialOptions();
  options.emit_certificate = true;
  const CertificationResult a =
      checker.CheckTrajectory(MakeBezier(start, end, 3, 0.0, 1.0), options);
  const CertificationResult b =
      checker.CheckTrajectory(MakeBezier(start, end, 3, -2.5, 4.2), options);

  EXPECT_EQ(a.verdict, b.verdict);
  EXPECT_EQ(a.stats.nodes, b.stats.nodes);
  EXPECT_EQ(a.stats.narrowphase_queries, b.stats.narrowphase_queries);
  EXPECT_EQ(a.stats.sphere_certifications, b.stats.sphere_certifications);
  EXPECT_EQ(a.stats.max_depth, b.stats.max_depth);

  ASSERT_TRUE(a.certificate.has_value());
  ASSERT_TRUE(b.certificate.has_value());
  ASSERT_EQ(a.certificate->records.size(), b.certificate->records.size());
  for (size_t i = 0; i < a.certificate->records.size(); ++i) {
    const CertificateRecord& ra = a.certificate->records[i];
    const CertificateRecord& rb = b.certificate->records[i];
    // The certified interval structure lives in parameter space, so it is
    // bit-identical; only the reported *times* would differ.
    EXPECT_EQ(ra.segment, rb.segment);
    EXPECT_EQ(ra.s_start, rb.s_start);
    EXPECT_EQ(ra.s_end, rb.s_end);
    EXPECT_EQ(ra.pair_index, rb.pair_index);
    EXPECT_EQ(ra.phi_hat, rb.phi_hat);
    EXPECT_EQ(ra.motion_bound, rb.motion_bound);
    EXPECT_EQ(ra.threshold, rb.threshold);
    EXPECT_TRUE(ra.qc == rb.qc);
  }
}

// ---------------------------------------------------------------------------
// 6. The node budget and breakpoint semantics.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, NodeBudgetExhausted) {
  const auto checker = MakeChecker(MakeArmWorld(), SerialOptions());
  Eigen::MatrixXd waypoints(3, 4);
  waypoints.col(0) = MakeQ(0.0, 0.0, 0.0);
  waypoints.col(1) = MakeQ(0.3, -0.1, 0.03);
  waypoints.col(2) = MakeQ(0.6, -0.3, 0.07);
  waypoints.col(3) = MakeQ(0.8, -0.4, 0.10);

  Options options = SerialOptions();
  options.max_nodes = 1;
  const CertificationResult result = checker.CheckPath(waypoints, options);
  EXPECT_EQ(result.verdict, Verdict::kBudgetExhausted);
  ASSERT_FALSE(result.findings.empty());
  // The remainder is reported as a non-definite finding at the earliest
  // uncovered time.
  EXPECT_FALSE(result.findings.front().definite);
  EXPECT_GE(result.findings.front().time, 0.0);
}

GTEST_TEST(CertifierTest, BreakpointWitnessesAreReported) {
  const auto checker = MakeChecker(MakeArmWorld(), SerialOptions());

  // q(t0) puts the arm straight into the post. Only the breakpoint pre-pass can
  // produce a witness *exactly* at t0; node midpoints are strictly interior.
  const CertificationResult at_start = checker.CheckTrajectory(
      MakeBezier(MakeQ(1.5708, 0.0, 0.0), MakeQ(0.5, 0.0, 0.0), 1));
  ASSERT_EQ(at_start.verdict, Verdict::kViolationFound);
  ASSERT_FALSE(at_start.findings.empty());
  const Finding& first = at_start.findings.front();
  EXPECT_EQ(first.time, 0.0);
  EXPECT_TRUE(first.definite);
  EXPECT_EQ(first.motion_bound, 0.0);
  EXPECT_LT(DistanceAtFinding(checker, first), kMargin);

  // A 3-waypoint path whose middle waypoint (the junction between segments, at
  // t = 1) is inside the post: the pre-pass must report the junction
  // configuration itself, not only interior node midpoints.
  Eigen::MatrixXd waypoints(3, 3);
  waypoints.col(0) = MakeQ(0.0, 0.0, 0.0);
  waypoints.col(1) = MakeQ(1.5708, 0.0, 0.0);
  waypoints.col(2) = MakeQ(3.0, 0.0, 0.0);
  const CertificationResult at_junction = checker.CheckPath(waypoints);
  ASSERT_EQ(at_junction.verdict, Verdict::kViolationFound);
  bool found_junction_witness = false;
  for (const Finding& finding : at_junction.findings) {
    if (finding.time == 1.0 && finding.definite &&
        finding.motion_bound == 0.0) {
      found_junction_witness = true;
      EXPECT_LT(DistanceAtFinding(checker, finding), kMargin);
    }
  }
  EXPECT_TRUE(found_junction_witness);
}

// ---------------------------------------------------------------------------
// 7. A small seeded soundness sweep. The full corpus (random worlds,
//    B-splines, 10^5 samples, hundreds of cases) lives in
//    test/soundness_fuzz_test.cc, which is timeout=long and opts out of asan
//    and lsan; this is the cheap standing guard that runs in every build
//    flavor.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, RandomTrajectoriesAreSoundAgainstDenseSampling) {
  const auto checker = MakeChecker(MakeArmWorld(), SerialOptions());
  std::mt19937 rng(1234);
  std::uniform_real_distribution<double> theta1(-3.0, 3.0);
  std::uniform_real_distribution<double> theta2(-2.0, 2.0);
  std::uniform_real_distribution<double> slide(0.0, 0.25);

  int certified = 0;
  int violating = 0;
  for (int trial = 0; trial < 15; ++trial) {
    Eigen::MatrixXd control_points(3, 4);
    for (int j = 0; j < 4; ++j) {
      control_points.col(j) << theta1(rng), theta2(rng), slide(rng);
    }
    const BezierCurve<double> trajectory(0.0, 1.0, control_points);
    const CertificationResult result = checker.CheckTrajectory(trajectory);
    const PiecewiseBezierPath path = checker.Normalize(trajectory);

    if (result.verdict == Verdict::kCertifiedFree) {
      ++certified;
      const SampledClearance sampled =
          SampleClearance(checker, path, 2000, kMargin);
      EXPECT_GT(sampled.min_clearance, kMargin)
          << "trial " << trial << " was certified but dense sampling found a "
          << "configuration at clearance " << sampled.min_clearance;
    }
    for (const Finding& finding : result.findings) {
      if (!finding.definite) continue;
      ++violating;
      // The witness must be exactly on the path and must really violate.
      EXPECT_LT((path.Value(finding.time) - finding.q).cwiseAbs().maxCoeff(),
                1e-9)
          << "trial " << trial;
      EXPECT_LT(DistanceAtFinding(checker, finding), kMargin)
          << "trial " << trial;
    }
  }
  // The corpus must actually exercise both outcomes.
  EXPECT_GT(certified, 0);
  EXPECT_GT(violating, 0);
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
