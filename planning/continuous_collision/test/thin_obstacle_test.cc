// A plate thin enough that Drake's SceneGraphCollisionChecker steps over it at
// its default edge_step_size; ContinuousCollisionChecker must reject the edge
// anyway. The mirror image is here too: a millimetre-scale gap that is
// genuinely free and certifies with a bounded node budget. Every world is built
// programmatically and no case uses an RNG or a model file.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/planning/collision_checker_params.h"
#include "drake/planning/continuous_collision/test/test_utilities.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::planning::CollisionCheckerParams;
using drake::planning::SceneGraphCollisionChecker;
using Eigen::Vector3d;
using Eigen::VectorXd;
using test::Box;
using test::DistanceAtFinding;
using test::Friction;
using test::Inertia;
using test::MakeCheckerPtr;
using test::MultibodyPlant;
using test::Parallelism;
using test::PrismaticJoint;
using test::RigidBody;
using test::RigidTransformd;
using test::RobotDiagram;
using test::RobotDiagramBuilder;
using test::Sphere;

// ---------------------------------------------------------------------------
// The geometry, and the arithmetic that makes default sampling blind to it.
// ---------------------------------------------------------------------------
//
// The robot is a 2-dof Cartesian gantry (prismatic x, then prismatic y)
// carrying a sphere of radius kToolRadius = 5 mm. Its configuration *is* the
// tool centre, which turns every number below into an exact, checkable
// statement about the sampled check.
//
// The edge runs from q1 = (-0.5, 0) to q2 = (+0.5, 0).
//
//   * Drake's default configuration distance (LinearDistanceAndInterpolation-
//     Provider with unit weights) is the Euclidean norm, so d(q1, q2) = 1.0 m
//     exactly.
//   * CollisionCheckerParams has *no* default edge_step_size: the field is
//     value-initialized to 0 and set_edge_step_size() rejects anything
//     non-positive, so "the default" is whatever the planning stack picks.
//     kDrakeEdgeStepSize = 0.05 is the value Drake's own planning tests and the
//     IRIS/GCS examples use, and for a 1 m edge it is generous.
//   * The checker therefore samples ⌈1.0 / 0.05⌉ = 20 uniform intervals, i.e.
//     21 configurations 0.05 m apart in x, at x = -0.50, -0.45, …, 0.00, 0.05,
//     …, 0.50. (Reconstructed and measured below.)
//   * The plate is kPlateThickness = 1 mm thick in x and welded at
//     x = kPlateX = 0.025, exactly halfway between the samples at x = 0.00 and
//     x = 0.05.
//   * Tool and plate are in contact for
//         |x − 0.025| ≤ kToolRadius + kPlateThickness/2 = 0.0055 m,
//     an interval 11 mm wide. 11 mm ≪ the 50 mm sample spacing, and the plate's
//     mid-plane sits 25 mm from the nearest sample, so that sample still
//     measures 25 − 5 − 0.5 = 19.5 mm of clearance.

constexpr double kToolRadius = 0.005;
constexpr double kPlateThickness = 0.001;
constexpr double kPlateX = 0.025;
constexpr double kDrakeEdgeStepSize = 0.05;
// Half-width, in x, of the set of configurations that touch the plate.
constexpr double kContactHalfWidth = kToolRadius + 0.5 * kPlateThickness;

// The gantry: q = (x, y) is the tool-sphere centre in the z = 0 plane. The
// robot lives in its own model instance so Drake's collision checker can be
// told which bodies are "the robot".
void AddGantry(MultibodyPlant<double>* plant) {
  const auto robot = plant->AddModelInstance("robot");
  const RigidBody<double>& carriage =
      plant->AddRigidBody("carriage", robot, Inertia());
  const RigidBody<double>& tool = plant->AddRigidBody("tool", robot, Inertia());
  plant->AddJoint<PrismaticJoint>("gantry_x", plant->world_body(), {}, carriage,
                                  {}, Vector3d::UnitX());
  plant->AddJoint<PrismaticJoint>("gantry_y", carriage, {}, tool, {},
                                  Vector3d::UnitY());
  plant->RegisterCollisionGeometry(tool, RigidTransformd(), Sphere(kToolRadius),
                                   "tool_geom", Friction());
}

void AddAnchoredBox(MultibodyPlant<double>* plant, const std::string& name,
                    const Vector3d& p_W, const Vector3d& full_size) {
  const auto env = plant->GetModelInstanceByName("env");
  const RigidBody<double>& body = plant->AddRigidBody(name, env, Inertia());
  plant->WeldFrames(plant->world_frame(), body.body_frame(),
                    RigidTransformd(p_W));
  plant->RegisterCollisionGeometry(
      body, RigidTransformd(), Box(full_size.x(), full_size.y(), full_size.z()),
      name + "_geom", Friction());
}

// One plate of the given thickness welded at x = `plate_x`, spanning 0.6 m in
// y and z so the tool cannot go around it.
std::unique_ptr<RobotDiagram<double>> MakePlateWorld(double plate_x,
                                                     double thickness) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  plant.AddModelInstance("env");
  AddGantry(&plant);
  AddAnchoredBox(&plant, "plate", Vector3d(plate_x, 0.0, 0.0),
                 Vector3d(thickness, 0.6, 0.6));
  return builder.Build();
}

// The mirrored world: a slot 2·`half_gap` wide in y formed by two thin plates,
// running along the whole of the tool's x travel.
std::unique_ptr<RobotDiagram<double>> MakeSlotWorld(double half_gap) {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  plant.AddModelInstance("env");
  AddGantry(&plant);
  AddAnchoredBox(&plant, "slot_left", Vector3d(0.0, half_gap, 0.0),
                 Vector3d(0.8, kPlateThickness, 0.6));
  AddAnchoredBox(&plant, "slot_right", Vector3d(0.0, -half_gap, 0.0),
                 Vector3d(0.8, kPlateThickness, 0.6));
  return builder.Build();
}

// Drake's sampled edge checker, always on its own freshly built RobotDiagram:
// SceneGraphCollisionChecker rewrites collision filters on the model it is
// handed, which would otherwise perturb the pair table
// ContinuousCollisionChecker snapshots at construction.
SceneGraphCollisionChecker MakeDrakeChecker(
    std::unique_ptr<RobotDiagram<double>> model, double edge_step_size) {
  std::shared_ptr<RobotDiagram<double>> shared(std::move(model));
  CollisionCheckerParams params;
  params.robot_model_instances = {
      shared->plant().GetModelInstanceByName("robot")};
  params.model = std::move(shared);
  params.edge_step_size = edge_step_size;
  // This test never calls the parallel entry points, and the default
  // (Parallelism::Max()) would allocate one context per hardware thread for
  // nothing.
  params.implicit_context_parallelism = Parallelism::None();
  return SceneGraphCollisionChecker(std::move(params));
}

Options CertifiedOptions() {
  Options options;
  options.margin = 0.0;
  options.parallelism = Parallelism::None();
  return options;
}

VectorXd MakeQ(double x, double y) {
  VectorXd q(2);
  q << x, y;
  return q;
}

Eigen::MatrixXd Waypoints(const VectorXd& q1, const VectorXd& q2) {
  Eigen::MatrixXd waypoints(q1.size(), 2);
  waypoints.col(0) = q1;
  waypoints.col(1) = q2;
  return waypoints;
}

// ---------------------------------------------------------------------------
// 1. Pin the failure mode: Drake's sampled checker reports the edge free.
// ---------------------------------------------------------------------------

GTEST_TEST(ThinObstacleTest, DrakeSampledCheckerMissesTheThinPlate) {
  const VectorXd q1 = MakeQ(-0.5, 0.0);
  const VectorXd q2 = MakeQ(0.5, 0.0);
  const SceneGraphCollisionChecker drake_checker = MakeDrakeChecker(
      MakePlateWorld(kPlateX, kPlateThickness), kDrakeEdgeStepSize);

  // The distance the sample count is derived from is exactly the edge length,
  // and with 1 mm of plate between the waypoints the sampled check still calls
  // the edge free.
  EXPECT_NEAR(drake_checker.ComputeConfigurationDistance(q1, q2), 1.0, 1e-15);
  EXPECT_TRUE(drake_checker.CheckEdgeCollisionFree(q1, q2))
      << "the premise of this test no longer holds on this Drake pin: "
         "default-resolution sampling now catches the 1 mm plate";

  // The sample grid is ⌈1.0/0.05⌉ = 20 uniform intervals, i.e. samples at
  // x = -0.5 + k/20, which are exactly the multiples of 0.05. Sliding the plate
  // across one sample period and comparing Drake's verdict against "caught iff
  // the plate's mid-plane is within the contact half-width of some multiple of
  // 0.05" measures that grid instead of assuming it.
  const auto gap_to_nearest_sample = [](double x) {
    return std::abs(x -
                    kDrakeEdgeStepSize * std::round(x / kDrakeEdgeStepSize));
  };
  for (int i = 0; i <= 20; ++i) {
    const double offset = i * (kDrakeEdgeStepSize / 20.0);
    SCOPED_TRACE("plate mid-plane at x = " + std::to_string(offset));
    const SceneGraphCollisionChecker probe = MakeDrakeChecker(
        MakePlateWorld(offset, kPlateThickness), kDrakeEdgeStepSize);
    EXPECT_EQ(probe.CheckEdgeCollisionFree(q1, q2),
              gap_to_nearest_sample(offset) > kContactHalfWidth)
        << "Drake's sample grid is not the one this test's arithmetic assumes";
  }

  // With the grid confirmed, walk it through Drake's own interpolation function
  // and measure the clearance at every sample. The nearest sample is 25 mm from
  // the plate's mid-plane, i.e. 19.5 mm of clearance.
  const int num_intervals =
      static_cast<int>(std::ceil(1.0 / kDrakeEdgeStepSize));
  double nearest_sample_gap = std::numeric_limits<double>::infinity();
  for (int k = 0; k <= num_intervals; ++k) {
    const double ratio = static_cast<double>(k) / num_intervals;
    const VectorXd q =
        drake_checker.InterpolateBetweenConfigurations(q1, q2, ratio);
    nearest_sample_gap = std::min(nearest_sample_gap, std::abs(q[0] - kPlateX));
    EXPECT_TRUE(drake_checker.CheckConfigCollisionFree(q))
        << "sample " << k << " at x = " << q[0];
  }
  EXPECT_NEAR(nearest_sample_gap, 0.025, 1e-12);
  EXPECT_GT(nearest_sample_gap, kContactHalfWidth)
      << "the plate must sit strictly between two samples";

  // The miss is a resolution gap, not a modelling one: shrink the step size and
  // the very same sampled checker finds the plate.
  EXPECT_FALSE(MakeDrakeChecker(MakePlateWorld(kPlateX, kPlateThickness), 0.002)
                   .CheckEdgeCollisionFree(q1, q2));
}

// ---------------------------------------------------------------------------
// 2. The certified checker returns a definite violation with a real witness.
// ---------------------------------------------------------------------------

GTEST_TEST(ThinObstacleTest, CertifiedCheckerCatchesTheThinPlate) {
  const VectorXd q1 = MakeQ(-0.5, 0.0);
  const VectorXd q2 = MakeQ(0.5, 0.0);
  const std::shared_ptr<const RobotDiagram<double>> model =
      MakePlateWorld(kPlateX, kPlateThickness);
  const auto checker = MakeCheckerPtr(model, CertifiedOptions());

  const Result result = checker->CheckEdge(q1, q2);
  ASSERT_EQ(result.verdict, Verdict::kViolationFound);
  ASSERT_TRUE(result.finding.has_value());

  const Finding& finding = *result.finding;
  ASSERT_EQ(finding.q.size(), 2);

  // The witness lies inside the plate-crossing parameter interval. CheckEdge
  // normalizes to one order-1 segment over t ∈ [0, 1] with q(t) = q1 + t·(q2 −
  // q1), so x(t) = -0.5 + t and the crossing interval is
  // t ∈ (0.5 + kPlateX − h, 0.5 + kPlateX + h) with h = kContactHalfWidth.
  EXPECT_GT(finding.time, 0.5 + kPlateX - kContactHalfWidth);
  EXPECT_LT(finding.time, 0.5 + kPlateX + kContactHalfWidth);
  EXPECT_LT(std::abs(finding.q[0] - kPlateX), kContactHalfWidth);
  EXPECT_NEAR(finding.q[1], 0.0, 1e-15);
  // The witness is exactly on the trajectory ...
  EXPECT_LT((MakeQ(-0.5 + finding.time, 0.0) - finding.q).cwiseAbs().maxCoeff(),
            1e-12);

  // ... and a direct distance query at the witness, from a context this run
  // never touched, confirms the contact.
  const double phi = DistanceAtFinding(*model, finding);
  EXPECT_LT(phi, 0.0) << "the witness must be a genuine interpenetration";
  EXPECT_NEAR(phi, finding.distance, 1e-12);
  EXPECT_TRUE(finding.nearest_a_W.has_value());
  EXPECT_TRUE(finding.nearest_b_W.has_value());

  // CheckPath over the same two waypoints makes the same statement.
  EXPECT_EQ(checker->CheckPath(Waypoints(q1, q2)).verdict,
            Verdict::kViolationFound);
}

// ---------------------------------------------------------------------------
// 3. The mirror image: a genuinely free 3 mm squeeze is certified cheaply.
// ---------------------------------------------------------------------------

GTEST_TEST(ThinObstacleTest, NarrowGapCertifiedWithBoundedNodeBudget) {
  // Slot half-width 8.5 mm against a 5 mm tool sphere and a 0.5 mm plate
  // half-thickness leaves exactly 3 mm of clearance on each side, constant over
  // the whole 0.6 m of travel.
  constexpr double kHalfGap = 0.0085;
  constexpr double kClearance = kHalfGap - 0.5 * kPlateThickness - kToolRadius;
  static_assert(kClearance > 0.0);

  const auto checker =
      MakeCheckerPtr(MakeSlotWorld(kHalfGap), CertifiedOptions());
  const VectorXd q1 = MakeQ(-0.3, 0.0);
  const VectorXd q2 = MakeQ(0.3, 0.0);

  // Sampling passes here too, and this time it is right; the certified checker
  // agrees without sampling.
  EXPECT_TRUE(MakeDrakeChecker(MakeSlotWorld(kHalfGap), kDrakeEdgeStepSize)
                  .CheckEdgeCollisionFree(q1, q2));

  const Result result = checker->CheckEdge(q1, q2);
  EXPECT_EQ(result.verdict, Verdict::kCertifiedFree);
  EXPECT_FALSE(result.finding.has_value());

  // Node budget. Only the prismatic x coordinate moves, so λ = 1 for the two
  // tool-vs-plate pairs and the motion bound at depth d is the node's half
  // width, 0.6 / 2^(d+1). Certification needs ϕ − τ − Δ > ε, i.e.
  //     0.6 / 2^(d+1) < 0.003 − 1e-6   =>   2^(d+1) > 200.1   =>   d = 7,
  // and a full binary tree to depth 7 has 2^8 − 1 = 255 nodes. Both slot pairs
  // certify at the same depth, so the whole recursion is that one tree. The
  // ceiling below is ~2.5× that: loose enough to survive a differently-tuned
  // prefilter, tight enough to catch a regression that made the search blow up.
  EXPECT_LT(result.num_nodes, uint64_t{640})
      << "certifying a 3 mm gap should cost O(log(travel / clearance)) depth, "
         "not a blow-up";
  EXPECT_GE(result.num_nodes, uint64_t{64})
      << "a 3 mm gap over 0.6 m of travel cannot be certified shallowly; if it "
         "could, the motion bound would be unsound";
}

// ---------------------------------------------------------------------------
// 4. Thickness sweep: where the resolution gap closes.
// ---------------------------------------------------------------------------

GTEST_TEST(ThinObstacleTest, ThicknessSweepBracketsTheResolutionGap) {
  // Held fixed: the plate's mid-plane at x = 0.025 (halfway between two Drake
  // samples) and the tool radius. The sampled checker can only see the plate
  // once the contact half-width reaches the 25 mm sample gap, i.e. once
  //     thickness/2 + kToolRadius ≥ 0.025  ⇔  thickness ≥ 0.040 m.
  // At thickness = 0.040 the nearest sample's signed distance is exactly 0, so
  // "collision" (ϕ < 0) there is decided by rounding; that is why the
  // crossover assertion is a window rather than an equality. The certified
  // verdict must be kViolationFound at every thickness in the sweep, since the
  // plate is crossed in all of them.
  const VectorXd q1 = MakeQ(-0.5, 0.0);
  const VectorXd q2 = MakeQ(0.5, 0.0);
  double first_caught = std::numeric_limits<double>::quiet_NaN();
  for (const double thickness : {0.001, 0.002, 0.005, 0.010, 0.020, 0.030,
                                 0.038, 0.040, 0.042, 0.050, 0.080}) {
    SCOPED_TRACE("thickness = " + std::to_string(thickness));
    const bool drake_free =
        MakeDrakeChecker(MakePlateWorld(kPlateX, thickness), kDrakeEdgeStepSize)
            .CheckEdgeCollisionFree(q1, q2);
    if (!drake_free && std::isnan(first_caught)) first_caught = thickness;
    EXPECT_EQ(
        MakeCheckerPtr(MakePlateWorld(kPlateX, thickness), CertifiedOptions())
            ->CheckEdge(q1, q2)
            .verdict,
        Verdict::kViolationFound);
  }
  // The crossover must land where the arithmetic above predicts, 2*(0.025 -
  // kToolRadius) = 0.04 m, otherwise the sweep is measuring something other
  // than the resolution gap.
  EXPECT_GT(first_caught, 0.03);
  EXPECT_LT(first_caught, 0.06);
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
