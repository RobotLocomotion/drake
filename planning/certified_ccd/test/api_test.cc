/// @file
/// T9 — API / UX (test plan T9: the joint-support and geometry-support
/// scopes, and the architecture).
///
/// Every refusal this library makes has to be *actionable*: the message must
/// name the joint, geometry, coordinate, index or size the caller has to go and
/// fix. These tests therefore assert on message content, not just that
/// something was thrown — a bare EXPECT_THROW would pass for a message reading
/// "error" and leave a user with nothing to act on.
///
/// Coverage notes for two items of test-plan T9:
///   * Python bindings do not exist yet, so the pydrake-style smoke
///     tests are out of scope here.
///   * An *unfinalized* plant cannot reach the checker through Drake's public
///     API on this pin: RobotDiagramBuilder::Build() finalizes the plant
///     unconditionally and RobotDiagram's constructor is private to the
///     builder, so there is no way to construct the input that guard rejects.
///     The guard is therefore defensive; the adjacent, reachable guards (null
///     model) are pinned instead. See NullModelIsRefused below.

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/parallelism.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/composite_trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/spatial_inertia.h"
#include "drake/planning/certified_ccd/certified_continuous_collision_checker.h"
#include "drake/planning/robot_diagram.h"
#include "drake/planning/robot_diagram_builder.h"

namespace drake {
namespace planning {
namespace certified_ccd {
namespace {

using drake::Parallelism;
using drake::geometry::Box;
using drake::geometry::GeometryInstance;
using drake::geometry::HalfSpace;
using drake::geometry::ProximityProperties;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::Joint;
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::planning::RobotDiagram;
using drake::planning::RobotDiagramBuilder;
using drake::trajectories::BezierCurve;
using drake::trajectories::CompositeTrajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;
using Eigen::Vector3d;
using Eigen::VectorXd;

CoulombFriction<double> Friction() {
  return CoulombFriction<double>(1.0, 1.0);
}

SpatialInertia<double> Inertia() {
  return SpatialInertia<double>::SolidSphereWithMass(1.0, 0.05);
}

/// Runs `call`, requires it to throw, and returns the message so the caller can
/// assert on the identifiers it must contain. Reports the actual message on
/// every failure path, so a message regression is diagnosable from the log.
template <typename Callable>
std::string ThrowMessage(Callable&& call) {
  try {
    call();
  } catch (const std::exception& error) {
    return error.what();
  }
  ADD_FAILURE() << "expected an exception, but the call returned normally";
  return {};
}

void ExpectContains(const std::string& haystack, const std::string& needle) {
  EXPECT_NE(haystack.find(needle), std::string::npos)
      << "the message did not mention '" << needle << "'.\nMessage was:\n"
      << haystack;
}

std::unique_ptr<CertifiedContinuousCollisionChecker> MakeChecker(
    std::shared_ptr<const RobotDiagram<double>> model) {
  CertifiedContinuousCollisionChecker::Params params;
  params.model = std::move(model);
  params.default_options.parallelism = Parallelism::None();
  return std::make_unique<CertifiedContinuousCollisionChecker>(params);
}

/// A planar 2-dof arm (revolute, prismatic) with one anchored obstacle: the
/// well-formed world the dimension / options / trajectory tests use.
std::unique_ptr<RobotDiagram<double>> MakeArmWorld() {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const RigidBody<double>& link = plant.AddRigidBody("link", Inertia());
  const RigidBody<double>& tool = plant.AddRigidBody("tool", Inertia());
  plant.AddJoint<RevoluteJoint>("shoulder", plant.world_body(), {}, link, {},
                                Vector3d::UnitZ());
  plant.AddJoint<PrismaticJoint>("slide", link,
                                 RigidTransformd(Vector3d(0.30, 0.0, 0.0)),
                                 tool, {}, Vector3d::UnitX());
  plant.RegisterCollisionGeometry(link, RigidTransformd(Vector3d(0.15, 0, 0)),
                                  Box(0.30, 0.05, 0.05), "link_geom",
                                  Friction());
  plant.RegisterCollisionGeometry(tool, RigidTransformd(), Sphere(0.04),
                                  "tool_geom", Friction());
  const RigidBody<double>& post = plant.AddRigidBody("post", Inertia());
  plant.WeldFrames(plant.world_frame(), post.body_frame(),
                   RigidTransformd(Vector3d(0.0, 0.60, 0.0)));
  plant.RegisterCollisionGeometry(post, RigidTransformd(), Sphere(0.08),
                                  "post_geom", Friction());
  return builder.Build();
}

/// A *floating* base body carrying a one-revolute arm, plus an anchored
/// obstacle. MultibodyPlant::Finalize() gives the free base a
/// QuaternionFloatingJoint, so q = [quaternion(4), position(3), elbow(1)].
std::unique_ptr<RobotDiagram<double>> MakeFloatingBaseWorld() {
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const RigidBody<double>& base = plant.AddRigidBody("base", Inertia());
  const RigidBody<double>& arm = plant.AddRigidBody("arm", Inertia());
  plant.AddJoint<RevoluteJoint>("elbow", base,
                                RigidTransformd(Vector3d(0.10, 0.0, 0.0)), arm,
                                {}, Vector3d::UnitZ());
  plant.RegisterCollisionGeometry(base, RigidTransformd(), Sphere(0.05),
                                  "base_geom", Friction());
  plant.RegisterCollisionGeometry(arm, RigidTransformd(Vector3d(0.12, 0, 0)),
                                  Box(0.24, 0.04, 0.04), "arm_geom",
                                  Friction());
  const RigidBody<double>& post = plant.AddRigidBody("post", Inertia());
  plant.WeldFrames(plant.world_frame(), post.body_frame(),
                   RigidTransformd(Vector3d(0.0, 0.90, 0.0)));
  plant.RegisterCollisionGeometry(post, RigidTransformd(), Sphere(0.06),
                                  "post_geom", Friction());
  return builder.Build();
}

/// The name Drake gave the quaternion floating joint it added at Finalize().
std::string FloatingJointName(const MultibodyPlant<double>& plant) {
  for (drake::multibody::JointIndex index : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(index);
    if (joint.type_name() == "quaternion_floating") return joint.name();
  }
  ADD_FAILURE() << "the plant has no quaternion floating joint";
  return {};
}

/// q for MakeFloatingBaseWorld(): identity quaternion, `p` for the base
/// position, `elbow` for the joint.
VectorXd FloatingQ(const Vector3d& p, double elbow) {
  VectorXd q(8);
  q << 1.0, 0.0, 0.0, 0.0, p.x(), p.y(), p.z(), elbow;
  return q;
}

// ---------------------------------------------------------------------------
// 1. Joint scope (the joint-support scope): quaternion bases, and the
// constant-coordinate
//    carve-out that makes them usable anyway.
// ---------------------------------------------------------------------------

GTEST_TEST(ApiTest, MovingQuaternionBaseThrowsNamingTheJoint) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeFloatingBaseWorld();
  const auto checker = MakeChecker(model);
  const std::string joint_name = FloatingJointName(model->plant());
  ASSERT_FALSE(joint_name.empty());

  // Move a *quaternion* coordinate: straight-line interpolation of quaternion
  // components is not a rotation-space geodesic, so the convex-hull motion
  // bound has no meaning and the library must refuse rather than guess.
  Eigen::MatrixXd points(8, 2);
  points.col(0) = FloatingQ(Vector3d::Zero(), 0.0);
  points.col(1) = FloatingQ(Vector3d::Zero(), 0.0);
  points(0, 1) = 0.7071067811865476;  // w
  points(3, 1) = 0.7071067811865476;  // z
  const std::string message = ThrowMessage([&]() {
    checker->CheckTrajectory(BezierCurve<double>(0.0, 1.0, points));
  });
  ExpectContains(message, joint_name);
  ExpectContains(message, "quaternion_floating");
  // The message must also point at the way out.
  ExpectContains(message, "constant-coordinate carve-out");

  // Translating the base is refused for the same reason (the coordinate belongs
  // to an excluded joint), and the message names the coordinate index.
  Eigen::MatrixXd translated(8, 2);
  translated.col(0) = FloatingQ(Vector3d::Zero(), 0.0);
  translated.col(1) = FloatingQ(Vector3d(0.2, 0.0, 0.0), 0.0);
  const std::string translate_message = ThrowMessage([&]() {
    checker->CheckTrajectory(BezierCurve<double>(0.0, 1.0, translated));
  });
  ExpectContains(translate_message, joint_name);
  ExpectContains(translate_message, "coordinate 4");
}

GTEST_TEST(ApiTest, ConstantQuaternionBaseIsAcceptedEndToEnd) {
  // The joint-support carve-out: a floating base whose pose is *constant* along
  // the trajectory is treated as welded, so a floating-base robot is fully
  // usable as long as the given trajectory does not move the base. This is the
  // end-to-end version of that promise — not just "does not throw", but a real
  // verdict with a real certificate.
  std::shared_ptr<const RobotDiagram<double>> model = MakeFloatingBaseWorld();
  const auto checker = MakeChecker(model);

  Options options;
  options.parallelism = Parallelism::None();
  options.emit_certificate = true;

  Eigen::MatrixXd points(8, 3);
  for (int j = 0; j < 3; ++j) {
    points.col(j) = FloatingQ(Vector3d(0.05, -0.10, 0.0), 0.0);
  }
  points(7, 1) = 0.35;  // Only the elbow moves.
  points(7, 2) = 0.70;
  const BezierCurve<double> trajectory(0.0, 1.0, points);

  const PiecewiseBezierPath path = checker->Normalize(trajectory, options);
  const std::vector<bool>& constant = path.constant_coordinates();
  ASSERT_EQ(constant.size(), 8u);
  for (int i = 0; i < 7; ++i) {
    EXPECT_TRUE(constant[i])
        << "base coordinate " << i << " should have been flagged constant";
  }
  EXPECT_FALSE(constant[7]);

  const CertificationResult result =
      checker->CheckTrajectory(trajectory, options);
  EXPECT_EQ(result.verdict, Verdict::kCertifiedFree);
  ASSERT_TRUE(result.certificate.has_value());
  EXPECT_TRUE(VerifyCertificate(*checker, path, *result.certificate));
}

// ---------------------------------------------------------------------------
// 2. Geometry scope (the geometry-support scope): rotating half spaces and
// deformables.
// ---------------------------------------------------------------------------

GTEST_TEST(ApiTest, RotatingHalfSpaceThrowsAtConstruction) {
  // A half space on a body that *rotates* relative to an unfiltered partner has
  // unbounded reach, so no finite λ exists for that pair. This must be refused
  // when the checker is built, not discovered mid-certification.
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const RigidBody<double>& blade = plant.AddRigidBody("blade", Inertia());
  plant.AddJoint<RevoluteJoint>("spin", plant.world_body(), {}, blade, {},
                                Vector3d::UnitX());
  plant.RegisterCollisionGeometry(blade, RigidTransformd(), HalfSpace(),
                                  "blade_halfspace", Friction());
  const RigidBody<double>& post = plant.AddRigidBody("post", Inertia());
  plant.WeldFrames(plant.world_frame(), post.body_frame(),
                   RigidTransformd(Vector3d(0.0, 0.5, 0.0)));
  plant.RegisterCollisionGeometry(post, RigidTransformd(), Sphere(0.05),
                                  "post_geom", Friction());
  std::shared_ptr<const RobotDiagram<double>> model = builder.Build();

  const std::string message = ThrowMessage([&]() {
    MakeChecker(model);
  });
  ExpectContains(message, "blade_halfspace");
  ExpectContains(message, "post_geom");
  ExpectContains(message, "spin");
  // ... and it must say what to do about it.
  ExpectContains(message, "Box");
}

GTEST_TEST(ApiTest, AnchoredHalfSpaceUnderARotatingArmIsAccepted) {
  // The complement, so the rule above is not read as "half spaces are
  // unsupported": the overwhelmingly common case — an anchored ground plane
  // under a rotating arm — is accepted, because λ then bounds the *arm's*
  // points and signed distance is symmetric.
  RobotDiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = builder.plant();
  const RigidBody<double>& link = plant.AddRigidBody("link", Inertia());
  plant.AddJoint<RevoluteJoint>("shoulder", plant.world_body(), {}, link, {},
                                Vector3d::UnitZ());
  plant.RegisterCollisionGeometry(link, RigidTransformd(Vector3d(0.15, 0, 0)),
                                  Box(0.30, 0.05, 0.05), "link_geom",
                                  Friction());
  const RigidBody<double>& ground = plant.AddRigidBody("ground", Inertia());
  plant.WeldFrames(plant.world_frame(), ground.body_frame(),
                   RigidTransformd(Vector3d(0.0, 0.0, -0.4)));
  plant.RegisterCollisionGeometry(ground, RigidTransformd(), HalfSpace(),
                                  "ground_halfspace", Friction());
  std::shared_ptr<const RobotDiagram<double>> model = builder.Build();

  const auto checker = MakeChecker(model);
  // The probe report is part of the UX: it must say how each pair is routed.
  const std::string report = checker->distance_oracle().support_report();
  ExpectContains(report, "HalfSpace");
  EXPECT_EQ(
      checker->CheckEdge(VectorXd::Constant(1, 0.0), VectorXd::Constant(1, 1.5))
          .verdict,
      Verdict::kCertifiedFree);
}

GTEST_TEST(ApiTest, DeformableGeometryIsRefusedNamingIt) {
  // Deformables are out of scope (the geometry-support scope): their motion is
  // not described by the plant's generalized positions, so no motion bound
  // exists for them at all. Registering one is possible on this Drake pin (the
  // plant must be discrete, which RobotDiagramBuilder's default time step
  // already is), so the refusal is exercised on a real model rather than argued
  // about.
  RobotDiagramBuilder<double> builder(0.01);
  MultibodyPlant<double>& plant = builder.plant();
  const RigidBody<double>& post = plant.AddRigidBody("post", Inertia());
  plant.WeldFrames(plant.world_frame(), post.body_frame(),
                   RigidTransformd(Vector3d(0.4, 0.0, 0.0)));
  plant.RegisterCollisionGeometry(post, RigidTransformd(), Sphere(0.05),
                                  "post_geom", Friction());
  const RigidBody<double>& link = plant.AddRigidBody("link", Inertia());
  plant.AddJoint<RevoluteJoint>("shoulder", plant.world_body(), {}, link, {},
                                Vector3d::UnitZ());
  plant.RegisterCollisionGeometry(link, RigidTransformd(Vector3d(0.15, 0, 0)),
                                  Box(0.30, 0.05, 0.05), "link_geom",
                                  Friction());

  auto instance = std::make_unique<GeometryInstance>(
      RigidTransformd(Vector3d(0.0, 0.5, 0.0)), std::make_unique<Sphere>(0.05),
      "squishy_blob");
  ProximityProperties properties;
  drake::geometry::AddContactMaterial(1e8, {}, Friction(), &properties);
  instance->set_proximity_properties(properties);
  drake::multibody::fem::DeformableBodyConfig<double> config;
  config.set_youngs_modulus(1e6);
  plant.mutable_deformable_model().RegisterDeformableBody(std::move(instance),
                                                          config, 0.05);
  std::shared_ptr<const RobotDiagram<double>> model = builder.Build();
  ASSERT_EQ(model->scene_graph()
                .model_inspector()
                .GetAllDeformableGeometryIds()
                .size(),
            1u);

  const std::string message = ThrowMessage([&]() {
    MakeChecker(model);
  });
  ExpectContains(message, "deformable");
  ExpectContains(message, "squishy_blob");
}

// ---------------------------------------------------------------------------
// 3. Dimensions (trajectory normalization; the architecture).
// ---------------------------------------------------------------------------

// The displacement lemma is proved in the separated regime only, so a
// negative effective threshold (margin + padding < 0) is outside what the
// checker can certify and must be rejected, not silently "certified".
GTEST_TEST(ApiTest, NegativeEffectiveThresholdIsRejected) {
  const auto checker = MakeChecker(MakeArmWorld());
  Options options;
  options.parallelism = Parallelism::None();
  options.margin = -0.01;
  const VectorXd q0 = VectorXd::Zero(2);
  const VectorXd q1 = VectorXd::Constant(2, 0.1);
  const std::string message = ThrowMessage([&]() {
    checker->CheckEdge(q0, q1, options);
  });
  ExpectContains(message, "negative");
  ExpectContains(message, "filter the pair");
}

GTEST_TEST(ApiTest, DimensionMismatchMessagesNameTheSizes) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeArmWorld();
  const auto checker = MakeChecker(model);
  ASSERT_EQ(model->plant().num_positions(), 2);

  const std::string path_message = ThrowMessage([&]() {
    checker->CheckPath(Eigen::MatrixXd::Zero(3, 4));
  });
  ExpectContains(path_message, "CheckPath");
  ExpectContains(path_message, "3 rows");
  ExpectContains(path_message, "2 generalized positions");
  ExpectContains(path_message, "waypoints are columns");

  const std::string edge_message = ThrowMessage([&]() {
    checker->CheckEdge(VectorXd::Zero(2), VectorXd::Zero(5));
  });
  ExpectContains(edge_message, "CheckEdge");
  ExpectContains(edge_message, "sizes 2 and 5");

  const std::string trajectory_message = ThrowMessage([&]() {
    checker->CheckTrajectory(
        BezierCurve<double>(0.0, 1.0, Eigen::MatrixXd::Zero(7, 3)));
  });
  ExpectContains(trajectory_message, "7 rows");
  ExpectContains(trajectory_message, "2 generalized positions");

  // A single waypoint is not a path.
  const std::string single_message = ThrowMessage([&]() {
    checker->CheckPath(Eigen::MatrixXd::Zero(2, 1));
  });
  ExpectContains(single_message, "at least 2 waypoints");
}

// ---------------------------------------------------------------------------
// 4. Trajectory validation (trajectory normalization).
// ---------------------------------------------------------------------------

GTEST_TEST(ApiTest, DiscontinuousTrajectoryThrowsNamingTheJunction) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeArmWorld();
  const auto checker = MakeChecker(model);

  Eigen::MatrixXd first(2, 2);
  first << 0.0, 0.3, 0.0, 0.05;
  Eigen::MatrixXd second(2, 2);
  // Coordinate 1 teleports by 0.4 m at the junction.
  second << 0.3, 0.6, 0.45, 0.50;
  std::vector<drake::copyable_unique_ptr<Trajectory<double>>> segments;
  segments.emplace_back(std::make_unique<BezierCurve<double>>(0.0, 1.0, first));
  segments.emplace_back(
      std::make_unique<BezierCurve<double>>(1.0, 2.0, second));
  const CompositeTrajectory<double> trajectory(std::move(segments));

  const std::string message = ThrowMessage([&]() {
    checker->CheckTrajectory(trajectory);
  });
  ExpectContains(message, "C0 discontinuity");
  ExpectContains(message, "segments 0 and 1");
  ExpectContains(message, "coordinate 1");
  ExpectContains(message, "continuity_tolerance");
}

GTEST_TEST(ApiTest, DegreeAboveConversionCapThrows) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeArmWorld();
  const auto checker = MakeChecker(model);

  // 13 interpolation nodes ⇒ one polynomial segment of degree 12, above the
  // default max_conversion_degree of 10.
  const int kNodes = 13;
  VectorXd times(kNodes);
  Eigen::MatrixXd samples(2, kNodes);
  for (int i = 0; i < kNodes; ++i) {
    times[i] = i;
    samples(0, i) = 0.1 * ((i % 3) - 1);
    samples(1, i) = 0.02 * ((i % 5) - 2);
  }
  const PiecewisePolynomial<double> trajectory =
      PiecewisePolynomial<double>::LagrangeInterpolatingPolynomial(times,
                                                                   samples);
  const std::string message = ThrowMessage([&]() {
    checker->CheckTrajectory(trajectory);
  });
  ExpectContains(message, "polynomial degree 12");
  ExpectContains(message, "max_conversion_degree");

  // Raising the cap deliberately is the documented escape hatch, and it works.
  Options options;
  options.parallelism = Parallelism::None();
  options.max_conversion_degree = 12;
  EXPECT_NO_THROW(checker->Normalize(trajectory, options));
}

GTEST_TEST(ApiTest, UnsupportedTrajectoryTypeThrowsNamingTheType) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeArmWorld();
  const auto checker = MakeChecker(model);
  const PiecewiseQuaternionSlerp<double> trajectory(
      std::vector<double>{0.0, 1.0},
      std::vector<Eigen::Quaternion<double>>{
          Eigen::Quaternion<double>::Identity(),
          Eigen::Quaternion<double>(0.7071067811865476, 0.0, 0.0,
                                    0.7071067811865476)});
  const std::string message = ThrowMessage([&]() {
    checker->CheckTrajectory(trajectory);
  });
  ExpectContains(message, "unsupported trajectory type");
  ExpectContains(message, "PiecewiseQuaternionSlerp");
  // The message must list what *is* accepted.
  ExpectContains(message, "BezierCurve");
  ExpectContains(message, "BsplineTrajectory");
}

GTEST_TEST(ApiTest, ContinuousRevoluteIndexOutOfRangeThrows) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeArmWorld();
  const auto checker = MakeChecker(model);
  Options options;
  options.parallelism = Parallelism::None();
  options.continuous_revolute_indices = {0, 5};

  Eigen::MatrixXd points(2, 2);
  points << 0.0, 0.2, 0.0, 0.05;
  const std::string message = ThrowMessage([&]() {
    checker->CheckTrajectory(BezierCurve<double>(0.0, 1.0, points), options);
  });
  ExpectContains(message, "continuous_revolute_indices contains 5,");
  ExpectContains(message, "2 generalized positions");

  // A negative index is out of range too.
  options.continuous_revolute_indices = {-1};
  const std::string negative_message = ThrowMessage([&]() {
    checker->CheckTrajectory(BezierCurve<double>(0.0, 1.0, points), options);
  });
  ExpectContains(negative_message, "continuous_revolute_indices contains -1,");
}

// ---------------------------------------------------------------------------
// 5. Options and construction.
// ---------------------------------------------------------------------------

GTEST_TEST(ApiTest, OptionsValidationMessagesAreActionable) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeArmWorld();
  const auto checker = MakeChecker(model);
  Eigen::MatrixXd points(2, 2);
  points << 0.0, 0.2, 0.0, 0.05;
  const BezierCurve<double> trajectory(0.0, 1.0, points);
  const auto check_with = [&](const Options& options) {
    return ThrowMessage([&]() {
      checker->CheckTrajectory(trajectory, options);
    });
  };

  Options options;
  options.parallelism = Parallelism::None();

  Options bad = options;
  bad.min_interval = 0.0;
  ExpectContains(check_with(bad), "min_interval");
  bad.min_interval = 2.0;
  ExpectContains(check_with(bad), "(0, 1]");

  bad = options;
  bad.max_reported_findings = 0;
  ExpectContains(check_with(bad), "max_reported_findings");

  bad = options;
  bad.query_tolerance = -1.0;
  ExpectContains(check_with(bad), "query_tolerance");

  bad = options;
  bad.certificate_slack = -1e-9;
  ExpectContains(check_with(bad), "certificate_slack");

  bad = options;
  bad.max_nodes = 0;
  ExpectContains(check_with(bad), "max_nodes");

  bad = options;
  bad.margin = std::numeric_limits<double>::quiet_NaN();
  ExpectContains(check_with(bad), "margin");
}

GTEST_TEST(ApiTest, NullModelIsRefused) {
  CertifiedContinuousCollisionChecker::Params params;
  const std::string message = ThrowMessage([&]() {
    CertifiedContinuousCollisionChecker checker(params);
  });
  ExpectContains(message, "Params::model is null");
  // The message points at the requirement the (unreachable-through-Drake's
  // public API) finalization guard also enforces.
  ExpectContains(message, "finalized");
}

GTEST_TEST(ApiTest, MaxReportedFindingsIsRespected) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeArmWorld();
  const auto checker = MakeChecker(model);
  Options options;
  options.parallelism = Parallelism::None();

  // Sweep the arm out past the post at θ ≈ π/2 with the tool extended and back
  // again: two segments, each with its own violating region, so kCertifyAll
  // (which drops a violating pair once per subtree) has more than one finding
  // to cap.
  Eigen::MatrixXd waypoints(2, 3);
  waypoints << 0.0, 2.4, 0.0, 0.25, 0.25, 0.25;

  const CertificationResult uncapped = checker->CheckPath(waypoints, options);
  ASSERT_EQ(uncapped.verdict, Verdict::kViolationFound);
  ASSERT_GE(uncapped.findings.size(), 2u);
  EXPECT_LE(static_cast<int>(uncapped.findings.size()),
            options.max_reported_findings);

  for (const int cap : {1, 2}) {
    SCOPED_TRACE("cap = " + std::to_string(cap));
    Options capped = options;
    capped.max_reported_findings = cap;
    const CertificationResult result = checker->CheckPath(waypoints, capped);
    EXPECT_EQ(result.verdict, Verdict::kViolationFound);
    // Exactly `cap`, not merely at most: the sink keeps the cap earliest
    // entries, and this run has more than `cap` of them. An "at most" assertion
    // would be satisfied by a regression that returned nothing, which would
    // also make the prefix check below vacuous.
    ASSERT_EQ(static_cast<int>(result.findings.size()), cap);
    // The cap keeps the *earliest* findings, so a capped run is a prefix of the
    // uncapped one — dropping the latest entry can never remove an earlier one.
    for (std::size_t i = 0; i < result.findings.size(); ++i) {
      EXPECT_EQ(result.findings[i].time, uncapped.findings[i].time);
      EXPECT_EQ(result.findings[i].definite, uncapped.findings[i].definite);
    }
  }
}

}  // namespace
}  // namespace certified_ccd
}  // namespace planning
}  // namespace drake
