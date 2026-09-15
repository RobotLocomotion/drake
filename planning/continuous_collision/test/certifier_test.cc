// End-to-end tests of the certifier core and the public facade on a focused,
// hand-built corpus. The large randomized corpus lives in
// test/soundness_fuzz_test.cc, the thread-count sweep in
// test/concurrency_test.cc and the API throw conditions in test/api_test.cc;
// none is duplicated here.
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

#include "drake/planning/continuous_collision/piecewise_bezier_path.h"
#include "drake/planning/continuous_collision/test/test_utilities.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;
using internal::PiecewiseBezierPath;
using test::BezierCurve;
using test::Box;
using test::DistanceAtFinding;
using test::Friction;
using test::HalfSpace;
using test::Inertia;
using test::MakeCheckerPtr;
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

PiecewiseBezierPath Normalize(const BezierCurve<double>& trajectory) {
  return PiecewiseBezierPath::FromTrajectory(trajectory, {});
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
SampledClearance SampleClearance(const RobotDiagram<double>& model,
                                 const PiecewiseBezierPath& path,
                                 int samples_per_segment, double threshold) {
  auto root = model.CreateDefaultContext();
  auto& plant_context = model.plant().GetMyMutableContextFromRoot(root.get());
  const auto& scene_graph = model.scene_graph();
  const internal::DistanceOracle oracle(model);
  SampledClearance result;
  for (int k = 0; k < static_cast<int>(path.segments().size()); ++k) {
    const internal::BezierSegment& segment = path.segments()[k];
    for (int i = 0; i <= samples_per_segment; ++i) {
      const double s = static_cast<double>(i) / samples_per_segment;
      model.plant().SetPositions(&plant_context, path.EvaluateSegment(k, s));
      const auto& query_object =
          scene_graph.get_query_output_port().Eval<QueryObject<double>>(
              scene_graph.GetMyContextFromRoot(*root));
      for (const internal::PairRecord& pair : oracle.pairs()) {
        const double phi = oracle.SignedDistance(query_object, pair);
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
  const auto model = MakeArmWorld();
  const auto checker = MakeCheckerPtr(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10), 3);

  const Result result = checker->CheckTrajectory(trajectory);
  EXPECT_EQ(result.verdict, Verdict::kCertifiedFree);
  EXPECT_FALSE(result.finding.has_value());
  EXPECT_GT(result.num_nodes, 0u);

  // Independent cross-check: 10^4 dense samples must all clear the margin.
  const SampledClearance sampled =
      SampleClearance(*model, Normalize(trajectory), 10000, kMargin);
  EXPECT_GT(sampled.min_clearance, kMargin);
  EXPECT_TRUE(std::isnan(sampled.first_crossing));

  // The same statement through the other two entry points.
  Eigen::MatrixXd waypoints(3, 3);
  waypoints.col(0) = MakeQ(0.0, 0.0, 0.0);
  waypoints.col(1) = MakeQ(0.4, -0.2, 0.05);
  waypoints.col(2) = MakeQ(0.8, -0.4, 0.10);
  EXPECT_EQ(checker->CheckPath(waypoints).verdict, Verdict::kCertifiedFree);
  EXPECT_EQ(
      checker->CheckEdge(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10)).verdict,
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
  const std::shared_ptr<const RobotDiagram<double>> model = builder.Build();
  const auto checker = MakeCheckerPtr(model, SerialOptions());
  // Only the prismatic coordinate moves: the tool slides through the gap.
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.0, 0.0, 0.20), 1);

  const Result result = checker->CheckTrajectory(trajectory);
  EXPECT_EQ(result.verdict, Verdict::kCertifiedFree);
  EXPECT_FALSE(result.finding.has_value());
  // A 5 mm gap over the margin against ~0.2 m of travel cannot be certified at
  // the root: the motion bound has to be tightened by subdivision.
  EXPECT_GE(result.num_nodes, 16u);

  const SampledClearance sampled =
      SampleClearance(*model, Normalize(trajectory), 10000, kMargin);
  EXPECT_GT(sampled.min_clearance, kMargin);
  EXPECT_LT(sampled.min_clearance, kMargin + 0.01);
}

// ---------------------------------------------------------------------------
// 2. A sweeping trajectory that hits an obstacle: the witness is exact, and it
//    is the earliest one.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, ViolationWitnessIsExactAndEarliest) {
  const auto model = MakeArmWorld();
  const auto checker = MakeCheckerPtr(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(2.0, 0.0, 0.0), 1);

  const Result result = checker->CheckTrajectory(trajectory);
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  ASSERT_TRUE(result.finding.has_value());
  const Finding& finding = *result.finding;
  EXPECT_TRUE(finding.nearest_a_W.has_value());
  EXPECT_TRUE(finding.nearest_b_W.has_value());

  // The witness is exactly on the trajectory, so re-evaluating the path at the
  // reported time must reproduce it; and re-querying the distance from a fresh
  // context must confirm the violation.
  const PiecewiseBezierPath path = Normalize(trajectory);
  EXPECT_LT((path.Value(finding.time) - finding.q).cwiseAbs().maxCoeff(), 1e-9);
  const double phi = DistanceAtFinding(*model, finding);
  EXPECT_LT(phi, kMargin);
  EXPECT_NEAR(phi, finding.distance, 1e-12);

  // The branch-and-bound recursion drives the reported witness to the earliest
  // violating time, which dense sampling brackets from above.
  const SampledClearance sampled =
      SampleClearance(*model, path, 10000, kMargin);
  ASSERT_FALSE(std::isnan(sampled.first_crossing));
  EXPECT_NEAR(finding.time, sampled.first_crossing, 5e-3);
  EXPECT_LE(finding.time, sampled.first_crossing + 1e-9);
}

GTEST_TEST(CertifierTest, GrazingTangencyIsInconclusive) {
  // A world built for exact tangency: with θ1 = θ2 = 0 held constant the tool
  // centre slides along +x through (0.80, 0, 0), where the "graze" sphere sits
  // at distance 0.11, which is exactly r_tool + r_graze + kMargin.
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  AddArm(&plant);
  AddWeldedSphere(&plant, "graze", Vector3d(0.80, 0.11, 0.0), 0.05);
  const std::shared_ptr<const RobotDiagram<double>> model = builder.Build();
  const auto checker = MakeCheckerPtr(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.0, 0.0, 0.20), 1);

  Options options = SerialOptions();
  // A coarse resolution keeps the cost of the tangency cascade bounded; the
  // verdict is what matters here, not the depth.
  options.distance_resolution = 1e-4;
  const Result result = checker->CheckTrajectory(trajectory, options);

  EXPECT_EQ(result.verdict, Verdict::kInconclusive);
  ASSERT_TRUE(result.finding.has_value());
  // The near-witness sits within a hair of the threshold.
  EXPECT_NEAR(result.finding->distance, kMargin, 1e-3);
  EXPECT_NEAR(DistanceAtFinding(*model, *result.finding),
              result.finding->distance, 1e-12);

  // Dense sampling confirms the tangency: the minimum clearance touches the
  // margin but (up to sampling) never dips meaningfully below it.
  const SampledClearance sampled =
      SampleClearance(*model, Normalize(trajectory), 10000, kMargin);
  EXPECT_NEAR(sampled.min_clearance, kMargin, 1e-6);
}

// ---------------------------------------------------------------------------
// 3. Breakpoints: violations exactly at t0 and at a junction. Node midpoints
//    are strictly interior, so only the breakpoint pre-pass can find these.
//    A static pair (one whose J(p) the constant-coordinate carve-out emptied)
//    is likewise resolved there, once, at q(t0).
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, BreakpointWitnessesAreReported) {
  const auto model = MakeArmWorld();
  const auto checker = MakeCheckerPtr(model, SerialOptions());

  // q(t0) puts the arm straight into the post.
  const Result at_start = checker->CheckTrajectory(
      MakeBezier(MakeQ(1.5708, 0.0, 0.0), MakeQ(0.5, 0.0, 0.0), 1));
  ASSERT_EQ(at_start.verdict, Verdict::kViolationFound);
  ASSERT_TRUE(at_start.finding.has_value());
  EXPECT_EQ(at_start.finding->time, 0.0);
  EXPECT_LT(DistanceAtFinding(*model, *at_start.finding), kMargin);

  // A 3-waypoint path whose middle waypoint (the junction between segments, at
  // t = 1) is inside the post, and whose first segment is free: the earliest
  // witness must be the junction configuration itself.
  Eigen::MatrixXd waypoints(3, 3);
  waypoints.col(0) = MakeQ(0.0, 0.0, 0.0);
  waypoints.col(1) = MakeQ(1.5708, 0.0, 0.0);
  waypoints.col(2) = MakeQ(3.0, 0.0, 0.0);
  const Result at_junction = checker->CheckPath(waypoints);
  ASSERT_EQ(at_junction.verdict, Verdict::kViolationFound);
  ASSERT_TRUE(at_junction.finding.has_value());
  EXPECT_LE(at_junction.finding->time, 1.0);
  EXPECT_LT(DistanceAtFinding(*model, *at_junction.finding), kMargin);
}

GTEST_TEST(CertifierTest, StaticPairsAreResolvedAtTheStart) {
  // MultibodyPlant::Finalize() already filters every pair *within* a welded
  // subgraph, so two anchored obstacles never even reach the checker as a
  // candidate pair. The reachable source of J(p) = ∅ is the
  // constant-coordinate carve-out: with θ1 and θ2 constant, the link1/link2
  // pair and every link-vs-obstacle pair stop depending on any moving
  // coordinate and are settled once, at q(t0).
  const auto model = MakeArmWorld();
  const auto checker = MakeCheckerPtr(model, SerialOptions());

  // Free: only the prismatic coordinate moves, away from every obstacle.
  EXPECT_EQ(checker
                ->CheckTrajectory(MakeBezier(MakeQ(0.3, -0.2, 0.0),
                                             MakeQ(0.3, -0.2, 0.15), 2))
                .verdict,
            Verdict::kCertifiedFree);

  // Violating: link1 is parked inside the pillar for the whole trajectory, so
  // the only witness available is the static-pair test at q(t0).
  const Result result = checker->CheckTrajectory(
      MakeBezier(MakeQ(2.575, 0.0, 0.0), MakeQ(2.575, 0.0, 0.15), 2));
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  ASSERT_TRUE(result.finding.has_value());
  EXPECT_EQ(result.finding->time, 0.0);
  EXPECT_LT(DistanceAtFinding(*model, *result.finding), kMargin);
}

// ---------------------------------------------------------------------------
// 4. Retiming invariance: the proof is a property of the path.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, RetimingInvariance) {
  const auto checker = MakeCheckerPtr(MakeArmWorld(), SerialOptions());
  const VectorXd start = MakeQ(0.0, 0.0, 0.0);
  const VectorXd end = MakeQ(0.8, -0.4, 0.10);

  const Result a =
      checker->CheckTrajectory(MakeBezier(start, end, 3, 0.0, 1.0));
  const Result b =
      checker->CheckTrajectory(MakeBezier(start, end, 3, -2.5, 4.2));

  // The recursion runs in the segment parameter; only the reported *times*
  // would ever differ, and on a certified run there are none.
  EXPECT_EQ(a.verdict, b.verdict);
  EXPECT_EQ(a.verdict, Verdict::kCertifiedFree);
  EXPECT_EQ(a.num_nodes, b.num_nodes);
}

// ---------------------------------------------------------------------------
// 5. A small seeded soundness sweep. The full corpus (random worlds,
//    B-splines, 10^5 samples, hundreds of cases) lives in
//    test/soundness_fuzz_test.cc, which is timeout=long and opts out of asan
//    and lsan; this is the cheap standing guard that runs in every build
//    flavor.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, RandomTrajectoriesAreSoundAgainstDenseSampling) {
  const auto model = MakeArmWorld();
  const auto checker = MakeCheckerPtr(model, SerialOptions());
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
    const Result result = checker->CheckTrajectory(trajectory);
    const PiecewiseBezierPath path = Normalize(trajectory);

    if (result.verdict == Verdict::kCertifiedFree) {
      ++certified;
      const SampledClearance sampled =
          SampleClearance(*model, path, 2000, kMargin);
      EXPECT_GT(sampled.min_clearance, kMargin)
          << "trial " << trial << " was certified but dense sampling found a "
          << "configuration at clearance " << sampled.min_clearance;
    } else if (result.verdict == Verdict::kViolationFound) {
      ++violating;
      // The witness must be exactly on the path and must really violate.
      const Finding& finding = *result.finding;
      EXPECT_LT((path.Value(finding.time) - finding.q).cwiseAbs().maxCoeff(),
                1e-9)
          << "trial " << trial;
      EXPECT_LT(DistanceAtFinding(*model, finding), kMargin)
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
