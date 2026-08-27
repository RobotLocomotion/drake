/// @file
/// End-to-end tests of the certifier core and the public facade (the test plan,
/// T4/T6/T7 restricted to a focused corpus; the large randomized T4 fuzz
/// corpus is a separate milestone and deliberately not duplicated here).
///
/// Every world is built programmatically, every trajectory is fixed, and every
/// cross-check is dense sampling of the *same* path the checker certified, so
/// the suite is deterministic and fast.

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/parallelism.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
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
using drake::geometry::HalfSpace;
using drake::geometry::QueryObject;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
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

constexpr double kMargin = 0.01;

CoulombFriction<double> Friction() {
  return CoulombFriction<double>(1.0, 1.0);
}

SpatialInertia<double> UnitInertia() {
  return SpatialInertia<double>::SolidSphereWithMass(1.0, 0.05);
}

/// A planar 3-dof arm (revolute, revolute, prismatic) in the z = 0 plane:
///
///   world --j1(Rz)--> link1 [box, x ∈ 0 .. 0.40]
///                       --j2(Rz @ x=0.40)--> link2 [box, x ∈ 0 .. 0.30]
///                            --j3(Px @ x=0.30)--> tool [sphere r = 0.05]
///
/// so q = (θ1, θ2, d) and the tool centre sits at radius ≈ 0.70 + d when the
/// arm is straight. Obstacles are welded to the world.
void AddArm(MultibodyPlant<double>* plant) {
  const RigidBody<double>& link1 = plant->AddRigidBody("link1", UnitInertia());
  const RigidBody<double>& link2 = plant->AddRigidBody("link2", UnitInertia());
  const RigidBody<double>& tool = plant->AddRigidBody("tool", UnitInertia());

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
  const RigidBody<double>& body = plant->AddRigidBody(name, UnitInertia());
  plant->WeldFrames(plant->world_frame(), body.body_frame(),
                    RigidTransformd(p_W));
  plant->RegisterCollisionGeometry(body, RigidTransformd(), Sphere(radius),
                                   name + "_geom", Friction());
}

/// The main world: the arm, two round obstacles at different sweep angles, a
/// ground halfspace (which exercises the analytic distance route and the
/// "skip the sphere prefilter" path) and a far ceiling box.
std::shared_ptr<const RobotDiagram<double>> MakeArmWorld() {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  AddArm(&plant);
  // Angle ≈ 1.571 rad from the arm's home direction, radius 0.75.
  AddWeldedSphere(&plant, "post", Vector3d(0.0, 0.75, 0.0), 0.10);
  // Angle ≈ 2.575 rad, radius 0.65.
  AddWeldedSphere(&plant, "pillar", Vector3d(-0.55, 0.35, 0.0), 0.08);

  const RigidBody<double>& ground = plant.AddRigidBody("ground", UnitInertia());
  plant.WeldFrames(plant.world_frame(), ground.body_frame(),
                   RigidTransformd(Vector3d(0.0, 0.0, -0.50)));
  plant.RegisterCollisionGeometry(ground, RigidTransformd(), HalfSpace(),
                                  "ground_geom", Friction());

  const RigidBody<double>& ceiling =
      plant.AddRigidBody("ceiling", UnitInertia());
  plant.WeldFrames(plant.world_frame(), ceiling.body_frame(),
                   RigidTransformd(Vector3d(0.0, 0.0, 0.90)));
  plant.RegisterCollisionGeometry(ceiling, RigidTransformd(),
                                  Box(2.0, 2.0, 0.20), "ceiling_geom",
                                  Friction());
  return std::shared_ptr<const RobotDiagram<double>>(builder.Build());
}

/// A world built for exact tangency: with θ1 = θ2 = 0 held constant the tool
/// centre slides along +x through (0.80, 0, 0), where the "graze" sphere sits
/// at distance 0.11 — exactly r_tool + r_graze + kMargin.
std::shared_ptr<const RobotDiagram<double>> MakeGrazeWorld() {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  AddArm(&plant);
  AddWeldedSphere(&plant, "graze", Vector3d(0.80, 0.11, 0.0), 0.05);
  return std::shared_ptr<const RobotDiagram<double>>(builder.Build());
}

/// A genuinely free squeeze: the tool slides between two spheres that leave
/// only 5 mm of clearance over the margin, so the certificate is real but has
/// to be earned by subdividing (the mirror image of the tangency world).
std::shared_ptr<const RobotDiagram<double>> MakeGapWorld() {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  AddArm(&plant);
  // Sphere surface to tool surface at the closest approach:
  // 0.115 − 0.05 − 0.05 = 0.015 = kMargin + 0.005.
  AddWeldedSphere(&plant, "gap_left", Vector3d(0.80, 0.115, 0.0), 0.05);
  AddWeldedSphere(&plant, "gap_right", Vector3d(0.80, -0.115, 0.0), 0.05);
  return std::shared_ptr<const RobotDiagram<double>>(builder.Build());
}

ContinuousCollisionChecker MakeChecker(
    std::shared_ptr<const RobotDiagram<double>> model, Options options) {
  ContinuousCollisionChecker::Params params;
  params.model = std::move(model);
  params.default_options = std::move(options);
  return ContinuousCollisionChecker(params);
}

Options SerialOptions() {
  Options options;
  options.margin = kMargin;
  options.parallelism = Parallelism::None();
  return options;
}

/// A cubic Bézier from `start` to `end` with linearly spaced control points
/// (so the curve is the straight segment, traversed with a nontrivial
/// parametrization) over the time interval [t0, t1].
BezierCurve<double> MakeBezier(const VectorXd& start, const VectorXd& end,
                               int order, double t0, double t1) {
  Eigen::MatrixXd control_points(start.size(), order + 1);
  for (int j = 0; j <= order; ++j) {
    const double u = static_cast<double>(j) / order;
    control_points.col(j) = (1.0 - u) * start + u * end;
  }
  return BezierCurve<double>(t0, t1, control_points);
}

/// Result of the dense-sampling cross-check.
struct SampledClearance {
  double min_clearance{std::numeric_limits<double>::infinity()};
  /// Time of the first sample whose clearance drops below `threshold`, or NaN.
  double first_crossing{std::numeric_limits<double>::quiet_NaN()};
};

/// Densely samples `path` and evaluates every unfiltered pair discretely. This
/// is the independent check the certifier's continuum claim is measured
/// against; it reuses the (separately tested, T3) distance oracle so that
/// halfspace pairs are handled the same way.
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

/// Re-evaluates one finding's configuration from scratch and returns the
/// oracle distance of its pair there.
double DistanceAtFinding(const ContinuousCollisionChecker& checker,
                         const Finding& finding) {
  const RobotDiagram<double>& model = checker.model();
  auto root = model.CreateDefaultContext();
  auto& plant_context = model.plant().GetMyMutableContextFromRoot(root.get());
  model.plant().SetPositions(&plant_context, finding.q);
  const auto& scene_graph = model.scene_graph();
  const auto& query_object =
      scene_graph.get_query_output_port().Eval<QueryObject<double>>(
          scene_graph.GetMyContextFromRoot(*root));
  for (const PairRecord& pair : checker.pairs()) {
    if (pair.id.a == finding.pair.a && pair.id.b == finding.pair.b) {
      return checker.distance_oracle().SignedDistance(query_object, pair);
    }
  }
  ADD_FAILURE() << "the finding names a pair the checker does not know.";
  return 0.0;
}

VectorXd MakeQ(double theta1, double theta2, double d) {
  VectorXd q(3);
  q << theta1, theta2, d;
  return q;
}

// ---------------------------------------------------------------------------
// 1. A free trajectory is certified, and dense sampling agrees.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, FreeTrajectoryCertified) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10), 3, 0.0, 1.0);

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
}

GTEST_TEST(CertifierTest, NarrowGapCertifiedBySubdivision) {
  const auto model = MakeGapWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  // Only the prismatic coordinate moves: the tool slides through the gap.
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.0, 0.0, 0.20), 1, 0.0, 1.0);

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

GTEST_TEST(CertifierTest, FreePathAndEdgeCertified) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());

  Eigen::MatrixXd waypoints(3, 3);
  waypoints.col(0) = MakeQ(0.0, 0.0, 0.0);
  waypoints.col(1) = MakeQ(0.4, -0.2, 0.05);
  waypoints.col(2) = MakeQ(0.8, -0.4, 0.10);
  const CertificationResult path_result = checker.CheckPath(waypoints);
  EXPECT_EQ(path_result.verdict, Verdict::kCertifiedFree);

  const CertificationResult edge_result =
      checker.CheckEdge(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10));
  EXPECT_EQ(edge_result.verdict, Verdict::kCertifiedFree);
}

// ---------------------------------------------------------------------------
// 2. A sweeping trajectory that hits an obstacle.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, ViolationFoundWithExactWitness) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(2.0, 0.0, 0.0), 1, 0.0, 1.0);

  const CertificationResult result = checker.CheckTrajectory(trajectory);
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  ASSERT_FALSE(result.findings.empty());
  const Finding& finding = result.findings.front();
  EXPECT_TRUE(finding.definite);
  EXPECT_TRUE(finding.nearest_a_W.has_value());
  EXPECT_TRUE(finding.nearest_b_W.has_value());

  // The witness is exactly on the trajectory, so re-evaluating the path at the
  // reported time must reproduce it.
  const PiecewiseBezierPath path = checker.Normalize(trajectory);
  EXPECT_LT((path.Value(finding.time) - finding.q).cwiseAbs().maxCoeff(), 1e-9);

  // ... and re-querying the distance from a fresh context must confirm the
  // violation.
  const double phi = DistanceAtFinding(checker, finding);
  EXPECT_LT(phi, kMargin);
  EXPECT_NEAR(phi, finding.distance, 1e-12);
}

GTEST_TEST(CertifierTest, FindFirstReturnsEarliestWitness) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(2.0, 0.0, 0.0), 1, 0.0, 1.0);

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

// ---------------------------------------------------------------------------
// 3. Grazing tangency is inconclusive — never certified free.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, GrazingTangencyIsInconclusive) {
  const auto model = MakeGrazeWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  // θ1 = θ2 = 0 throughout; only the prismatic coordinate moves, sliding the
  // tool sphere past the obstacle at exactly margin distance.
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.0, 0.0, 0.20), 1, 0.0, 1.0);

  Options options = SerialOptions();
  // A coarser floor keeps the cost of the tangency cascade bounded; the
  // verdict is what matters here, not the depth.
  options.min_interval = 1e-4;
  const CertificationResult result =
      checker.CheckTrajectory(trajectory, options);

  EXPECT_EQ(result.verdict, Verdict::kInconclusive);
  EXPECT_NE(result.verdict, Verdict::kCertifiedFree);
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
// 4. Static pairs (J(p) = ∅) are resolved once and certified globally.
// ---------------------------------------------------------------------------

// Note on where static pairs come from: MultibodyPlant::Finalize() already
// filters every pair *within* a welded subgraph, so two anchored obstacles (or
// two members of a welded cluster on the robot) never even reach the checker
// as a candidate pair. The reachable source of J(p) = ∅ is therefore the
// constant-coordinate carve-out of trajectory normalization; the joint-support
// scope: a coordinate that no control point of the trajectory moves is removed
// from every J(p), and pairs left with an empty set are resolved once at q(t0).
GTEST_TEST(CertifierTest, StaticPairsResolvedOnce) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  // Only the prismatic coordinate moves: θ1 and θ2 are constant, so every pair
  // whose relative pose depends only on them becomes static.
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.3, -0.2, 0.0), MakeQ(0.3, -0.2, 0.15), 2, 0.0, 1.0);

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

  // A static pair is certified exactly once — one full-segment record per
  // segment, all sharing the single representative configuration q(t0) — and
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
}

// ---------------------------------------------------------------------------
// 4b. Padding reaches the effective threshold, and the env/self split is the
//     documented one (self = both bodies move relative to the world).
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, PaddingSemantics) {
  const auto model = MakeArmWorld();
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10), 3, 0.0, 1.0);
  const auto is_arm_self_pair = [&model](const Finding& finding) {
    const auto& plant = model->plant();
    const std::string a = plant.get_body(finding.pair.body_a).name();
    const std::string b = plant.get_body(finding.pair.body_b).name();
    return (a == "link1" || a == "link2" || a == "tool") &&
           (b == "link1" || b == "link2" || b == "tool");
  };

  // The one robot-vs-robot pair (link1, tool) keeps ≈ 0.27 m of clearance on
  // this trajectory, so 0.4 m of *self* padding must break it — and nothing
  // else, because every other pair has an anchored side and takes the (zero)
  // environment padding.
  {
    ContinuousCollisionChecker::Params params;
    params.model = model;
    params.default_options = SerialOptions();
    params.padding.self_padding = 0.40;
    const ContinuousCollisionChecker checker(params);
    const CertificationResult result = checker.CheckTrajectory(trajectory);
    ASSERT_EQ(result.verdict, Verdict::kViolationFound);
    for (const Finding& finding : result.findings) {
      EXPECT_TRUE(is_arm_self_pair(finding))
          << "self padding must not apply to environment pairs";
    }
  }

  // Mirrored: environment padding reaches the arm-vs-obstacle pairs (the
  // ground halfspace is 0.45 m away) and leaves the self pair alone.
  {
    ContinuousCollisionChecker::Params params;
    params.model = model;
    params.default_options = SerialOptions();
    params.padding.env_padding = 0.50;
    const ContinuousCollisionChecker checker(params);
    const CertificationResult result = checker.CheckTrajectory(trajectory);
    ASSERT_EQ(result.verdict, Verdict::kViolationFound);
    for (const Finding& finding : result.findings) {
      EXPECT_FALSE(is_arm_self_pair(finding))
          << "environment padding must not apply to robot self pairs";
    }
  }

  // A per-body-pair matrix overrides the scalars.
  {
    ContinuousCollisionChecker::Params params;
    params.model = model;
    params.default_options = SerialOptions();
    params.padding.env_padding = 0.50;
    params.padding.per_body_pair = Eigen::MatrixXd::Zero(
        model->plant().num_bodies(), model->plant().num_bodies());
    const ContinuousCollisionChecker checker(params);
    EXPECT_EQ(checker.CheckTrajectory(trajectory).verdict,
              Verdict::kCertifiedFree);
  }

  // A mis-sized matrix is a clear throw.
  {
    ContinuousCollisionChecker::Params params;
    params.model = model;
    params.default_options = SerialOptions();
    params.padding.per_body_pair = Eigen::MatrixXd::Zero(2, 2);
    EXPECT_THROW(ContinuousCollisionChecker{params}, std::exception);
  }
}

// ---------------------------------------------------------------------------
// 5. Retiming invariance (T6): the certificate is a property of the path.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, RetimingInvariance) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  const VectorXd start = MakeQ(0.0, 0.0, 0.0);
  const VectorXd end = MakeQ(0.8, -0.4, 0.10);
  const BezierCurve<double> fast = MakeBezier(start, end, 3, 0.0, 1.0);
  const BezierCurve<double> slow = MakeBezier(start, end, 3, -2.5, 4.2);

  Options options = SerialOptions();
  options.emit_certificate = true;
  const CertificationResult a = checker.CheckTrajectory(fast, options);
  const CertificationResult b = checker.CheckTrajectory(slow, options);

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
// 6. Certificate emission, replay and mutation (T7).
// ---------------------------------------------------------------------------

class CertificateFixture : public ::testing::Test {
 protected:
  CertificateFixture()
      : model_(MakeArmWorld()),
        checker_(MakeChecker(model_, SerialOptions())),
        trajectory_(MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10), 3,
                               0.0, 1.0)),
        path_(checker_.Normalize(trajectory_)) {
    Options options = SerialOptions();
    options.emit_certificate = true;
    result_ = checker_.CheckTrajectory(trajectory_, options);
  }

  /// Index of a record belonging to a pair the trajectory actually moves (so
  /// the record carries a real node interval, not the global static one).
  int MovingRecordIndex() const {
    const MotionBoundTable table = checker_.ComputeMotionBounds(path_);
    for (int i = 0; i < static_cast<int>(result_.certificate->records.size());
         ++i) {
      const CertificateRecord& record = result_.certificate->records[i];
      if (!table.pair_is_static(record.pair_index) && record.s_end < 1.0) {
        return i;
      }
    }
    return -1;
  }

  std::shared_ptr<const RobotDiagram<double>> model_;
  ContinuousCollisionChecker checker_;
  BezierCurve<double> trajectory_;
  PiecewiseBezierPath path_;
  CertificationResult result_;
};

TEST_F(CertificateFixture, VerifiesOnACertifiedRun) {
  ASSERT_EQ(result_.verdict, Verdict::kCertifiedFree);
  ASSERT_TRUE(result_.certificate.has_value());
  EXPECT_FALSE(result_.certificate->records.empty());
  EXPECT_TRUE(VerifyCertificate(checker_, path_, *result_.certificate));
}

TEST_F(CertificateFixture, RejectsShrunkClearance) {
  Certificate certificate = *result_.certificate;
  ASSERT_FALSE(certificate.records.empty());
  certificate.records[0].phi_hat = certificate.records[0].threshold;
  EXPECT_FALSE(VerifyCertificate(checker_, path_, certificate));
}

TEST_F(CertificateFixture, RejectsInflatedClearance) {
  Certificate certificate = *result_.certificate;
  ASSERT_FALSE(certificate.records.empty());
  certificate.records[0].phi_hat += 1.0;
  EXPECT_FALSE(VerifyCertificate(checker_, path_, certificate));
}

TEST_F(CertificateFixture, RejectsWidenedInterval) {
  Certificate certificate = *result_.certificate;
  const int index = MovingRecordIndex();
  ASSERT_GE(index, 0);
  CertificateRecord& record = certificate.records[index];
  record.s_end = std::min(1.0, record.s_end + (record.s_end - record.s_start));
  EXPECT_FALSE(VerifyCertificate(checker_, path_, certificate));
}

TEST_F(CertificateFixture, RejectsTamperedRepresentativeConfiguration) {
  Certificate certificate = *result_.certificate;
  const int index = MovingRecordIndex();
  ASSERT_GE(index, 0);
  certificate.records[index].qc[0] += 0.1;
  EXPECT_FALSE(VerifyCertificate(checker_, path_, certificate));
}

TEST_F(CertificateFixture, RejectsDroppedCoverage) {
  Certificate certificate = *result_.certificate;
  const int index = MovingRecordIndex();
  ASSERT_GE(index, 0);
  certificate.records.erase(certificate.records.begin() + index);
  EXPECT_FALSE(VerifyCertificate(checker_, path_, certificate));
}

TEST_F(CertificateFixture, RejectsLoweredThreshold) {
  Certificate certificate = *result_.certificate;
  ASSERT_GE(certificate.records.size(), 2u);
  certificate.records[0].threshold -= 0.005;
  EXPECT_FALSE(VerifyCertificate(checker_, path_, certificate));
}

TEST_F(CertificateFixture, RejectsUniformlyLoweredThresholds) {
  // Self-consistency is not enough: a certificate whose records *all* agree on
  // a threshold nobody asked for proves a claim nobody asked for.
  Certificate certificate = *result_.certificate;
  ASSERT_FALSE(certificate.records.empty());
  for (CertificateRecord& record : certificate.records) {
    record.threshold = -1e9;
  }
  EXPECT_FALSE(VerifyCertificate(checker_, path_, certificate));
}

GTEST_TEST(CertifierTest, CertificateRejectsRebasedStaticRecord) {
  // A static record must be measured at the path's own start configuration:
  // "static" is relative to the constant-coordinate carve-out, so a record
  // re-based onto an off-path configuration would measure a different pair
  // pose entirely.
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.3, -0.2, 0.0), MakeQ(0.3, -0.2, 0.15), 2, 0.0, 1.0);
  Options options = SerialOptions();
  options.emit_certificate = true;
  const CertificationResult result =
      checker.CheckTrajectory(trajectory, options);
  ASSERT_EQ(result.verdict, Verdict::kCertifiedFree);
  ASSERT_TRUE(result.certificate.has_value());

  const PiecewiseBezierPath path = checker.Normalize(trajectory);
  const MotionBoundTable table = checker.ComputeMotionBounds(path);
  EXPECT_TRUE(VerifyCertificate(checker, path, *result.certificate));

  // Both directions: rotating θ1 toward the obstacles reduces the clearance
  // the replay measures, while rotating away *increases* it — the case only
  // the "static records are pinned to q(t0)" check can catch.
  for (const double delta : {1.5, -1.5}) {
    Certificate certificate = *result.certificate;
    int tampered = 0;
    for (CertificateRecord& record : certificate.records) {
      if (table.pair_is_static(record.pair_index)) {
        // θ1 is a coordinate this path holds constant, hence one the carve-out
        // removed from J(p), but one that certainly moves the pair.
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
// 7. Search modes, finding caps and the node budget.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, CertifyAllReportsEveryViolation) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  // Sweeping θ1 from 0 to 3 rad passes the post (≈1.57 rad) and then the
  // pillar (≈2.58 rad): two disjoint violating regions, different pairs.
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(3.0, 0.0, 0.0), 1, 0.0, 1.0);

  const CertificationResult result = checker.CheckTrajectory(trajectory);
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  ASSERT_GE(result.findings.size(), 2u);
  for (size_t i = 1; i < result.findings.size(); ++i) {
    EXPECT_LE(result.findings[i - 1].time, result.findings[i].time)
        << "findings must be earliest-first";
  }
  int definite = 0;
  for (const Finding& finding : result.findings) {
    if (finding.definite) {
      ++definite;
      EXPECT_LT(DistanceAtFinding(checker, finding), kMargin);
    }
  }
  EXPECT_GE(definite, 2);

  Options capped = SerialOptions();
  capped.max_reported_findings = 1;
  const CertificationResult capped_result =
      checker.CheckTrajectory(trajectory, capped);
  EXPECT_EQ(capped_result.verdict, Verdict::kViolationFound);
  EXPECT_EQ(capped_result.findings.size(), 1u);
  EXPECT_NEAR(capped_result.findings.front().time, result.findings.front().time,
              1e-12);
}

GTEST_TEST(CertifierTest, NodeBudgetExhausted) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
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

// ---------------------------------------------------------------------------
// 8. Parallel smoke: same verdict and same witness as serial.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, ParallelMatchesSerialOnFreeTrajectory) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10), 3, 0.0, 1.0);

  const CertificationResult serial = checker.CheckTrajectory(trajectory);
  Options parallel_options = SerialOptions();
  parallel_options.parallelism = Parallelism(4);
  const CertificationResult parallel =
      checker.CheckTrajectory(trajectory, parallel_options);

  EXPECT_EQ(serial.verdict, parallel.verdict);
  EXPECT_EQ(parallel.verdict, Verdict::kCertifiedFree);
  EXPECT_TRUE(parallel.findings.empty());
  // The same tree is explored either way; only the order differs.
  EXPECT_EQ(serial.stats.nodes, parallel.stats.nodes);
  EXPECT_EQ(serial.stats.narrowphase_queries,
            parallel.stats.narrowphase_queries);
}

GTEST_TEST(CertifierTest, ParallelMatchesSerialOnViolation) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(2.0, 0.0, 0.0), 1, 0.0, 1.0);

  Options serial_options = SerialOptions();
  serial_options.mode = SearchMode::kFindFirstViolation;
  Options parallel_options = serial_options;
  parallel_options.parallelism = Parallelism(4);

  const CertificationResult serial =
      checker.CheckTrajectory(trajectory, serial_options);
  const CertificationResult parallel =
      checker.CheckTrajectory(trajectory, parallel_options);

  ASSERT_EQ(serial.verdict, Verdict::kViolationFound);
  ASSERT_EQ(parallel.verdict, Verdict::kViolationFound);
  ASSERT_EQ(serial.findings.size(), 1u);
  ASSERT_EQ(parallel.findings.size(), 1u);
  // The earliest witness is deterministic across thread counts (the stats are
  // not).
  EXPECT_NEAR(serial.findings.front().time, parallel.findings.front().time,
              1e-12);
  EXPECT_LT((serial.findings.front().q - parallel.findings.front().q)
                .cwiseAbs()
                .maxCoeff(),
            1e-12);
}

GTEST_TEST(CertifierTest, ConcurrentChecksAreIndependent) {
  // The Check* methods are const and documented thread-safe: concurrent calls
  // must lease disjoint contexts from the pool (the full T8 sweep is a later
  // milestone; this is the smoke test for the lease).
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  const BezierCurve<double> free_trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.8, -0.4, 0.10), 3, 0.0, 1.0);
  const BezierCurve<double> bad_trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(2.0, 0.0, 0.0), 1, 0.0, 1.0);

  Options options = SerialOptions();
  options.parallelism = Parallelism(2);
  std::vector<Verdict> verdicts(8);
  std::vector<std::thread> threads;
  for (int i = 0; i < 8; ++i) {
    threads.emplace_back([&, i]() {
      verdicts[i] =
          (i % 2 == 0)
              ? checker.CheckTrajectory(free_trajectory, options).verdict
              : checker.CheckTrajectory(bad_trajectory, options).verdict;
    });
  }
  for (std::thread& thread : threads) thread.join();
  for (int i = 0; i < 8; ++i) {
    EXPECT_EQ(verdicts[i], (i % 2 == 0) ? Verdict::kCertifiedFree
                                        : Verdict::kViolationFound);
  }
}

// ---------------------------------------------------------------------------
// 9. Breakpoint semantics.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, ViolationExactlyAtStartTime) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  // q(t0) puts the arm straight into the post.
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(1.5708, 0.0, 0.0), MakeQ(0.5, 0.0, 0.0), 1, 0.0, 1.0);

  const CertificationResult result = checker.CheckTrajectory(trajectory);
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  ASSERT_FALSE(result.findings.empty());
  const Finding& finding = result.findings.front();
  // Only the breakpoint pre-pass can produce a witness *exactly* at t0; node
  // midpoints are strictly interior.
  EXPECT_EQ(finding.time, 0.0);
  EXPECT_TRUE(finding.definite);
  EXPECT_EQ(finding.motion_bound, 0.0);
  EXPECT_LT(DistanceAtFinding(checker, finding), kMargin);
}

GTEST_TEST(CertifierTest, ViolationAtAJunctionIsReported) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  // A 3-waypoint path whose middle waypoint — the junction between segments,
  // at t = 1 — is inside the post.
  Eigen::MatrixXd waypoints(3, 3);
  waypoints.col(0) = MakeQ(0.0, 0.0, 0.0);
  waypoints.col(1) = MakeQ(1.5708, 0.0, 0.0);
  waypoints.col(2) = MakeQ(3.0, 0.0, 0.0);

  const CertificationResult result = checker.CheckPath(waypoints);
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  bool found_junction_witness = false;
  for (const Finding& finding : result.findings) {
    if (finding.time == 1.0 && finding.definite &&
        finding.motion_bound == 0.0) {
      found_junction_witness = true;
      EXPECT_LT(DistanceAtFinding(checker, finding), kMargin);
    }
  }
  EXPECT_TRUE(found_junction_witness)
      << "the breakpoint pre-pass must report the junction configuration "
         "itself, not only interior node midpoints";
}

// ---------------------------------------------------------------------------
// 9b. A small seeded soundness sweep. The full T4 corpus (random worlds,
//     B-splines, 10^5 samples, hundreds of cases) is a separate milestone;
//     this is the cheap standing guard that no kCertifiedFree of *this* driver
//     survives dense sampling, and that every definite witness really violates.
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, RandomTrajectoriesAreSoundAgainstDenseSampling) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
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

// ---------------------------------------------------------------------------
// 10. API guardrails (the full T9 suite lives in api_test).
// ---------------------------------------------------------------------------

GTEST_TEST(CertifierTest, ApiThrowsOnDimensionMismatch) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());

  Eigen::MatrixXd wrong_rows(2, 3);
  wrong_rows.setZero();
  EXPECT_THROW(checker.CheckPath(wrong_rows), std::exception);

  EXPECT_THROW(checker.CheckEdge(VectorXd::Zero(2), VectorXd::Zero(3)),
               std::exception);

  Eigen::MatrixXd control_points(5, 2);
  control_points.setZero();
  const BezierCurve<double> wrong_trajectory(0.0, 1.0, control_points);
  EXPECT_THROW(checker.CheckTrajectory(wrong_trajectory), std::exception);

  // A single waypoint is not a path.
  Eigen::MatrixXd single(3, 1);
  single.setZero();
  EXPECT_THROW(checker.CheckPath(single), std::exception);
}

GTEST_TEST(CertifierTest, ApiThrowsOnBadOptions) {
  const auto model = MakeArmWorld();
  const auto checker = MakeChecker(model, SerialOptions());
  const BezierCurve<double> trajectory =
      MakeBezier(MakeQ(0.0, 0.0, 0.0), MakeQ(0.1, 0.0, 0.0), 1, 0.0, 1.0);

  Options bad = SerialOptions();
  bad.min_interval = 0.0;
  EXPECT_THROW(checker.CheckTrajectory(trajectory, bad), std::exception);

  bad = SerialOptions();
  bad.max_reported_findings = 0;
  EXPECT_THROW(checker.CheckTrajectory(trajectory, bad), std::exception);

  bad = SerialOptions();
  bad.query_tolerance = -1.0;
  EXPECT_THROW(checker.CheckTrajectory(trajectory, bad), std::exception);
}

GTEST_TEST(CertifierTest, ConstructorRejectsNullModel) {
  ContinuousCollisionChecker::Params params;
  EXPECT_THROW(ContinuousCollisionChecker{params}, std::exception);
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
