// Which joints, geometries, dimensions and options the checker accepts, and
// what it says when it refuses. A refusal must name the joint, geometry,
// coordinate, index or size the caller has to go and fix, so these tests assert
// on message content: a bare EXPECT_THROW would pass for a message reading
// "error".
//
// The refusals that belong to trajectory normalization (junction
// discontinuity, degree cap, unsupported trajectory type, out-of-range
// continuous-revolute index) are asserted, with the same message identifiers,
// in test/piecewise_bezier_path_test.cc. The pydrake surface is covered in
// bindings/pydrake/planning/test/continuous_collision_test.py.

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/tree/joint.h"
#include "drake/planning/continuous_collision/test/test_utilities.h"

namespace drake {
namespace planning {
namespace continuous_collision {
namespace {

using drake::geometry::GeometryInstance;
using drake::geometry::ProximityProperties;
using drake::multibody::Joint;
using Eigen::Vector3d;
using Eigen::VectorXd;
using test::BezierCurve;
using test::Box;
using test::Friction;
using test::HalfSpace;
using test::Inertia;
using test::MakeCheckerPtr;
using test::MultibodyPlant;
using test::Parallelism;
using test::PrismaticJoint;
using test::RevoluteJoint;
using test::RigidBody;
using test::RigidTransformd;
using test::RobotDiagram;
using test::RobotDiagramBuilder;
using test::Sphere;
using test::ThrowMessage;
using ::testing::AllOf;
using ::testing::HasSubstr;

Options SerialOptions() {
  Options options;
  options.parallelism = Parallelism::None();
  return options;
}

// A planar 2-dof arm (revolute, prismatic) with one anchored obstacle: the
// well-formed world the dimension / options / trajectory tests use.
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

// A floating base body carrying a one-revolute arm, plus an anchored obstacle.
// MultibodyPlant::Finalize() gives the free base a QuaternionFloatingJoint, so
// q = [quaternion(4), position(3), elbow(1)].
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

// The name Drake gave the quaternion floating joint it added at Finalize().
std::string FloatingJointName(const MultibodyPlant<double>& plant) {
  for (drake::multibody::JointIndex index : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(index);
    if (joint.type_name() == "quaternion_floating") return joint.name();
  }
  ADD_FAILURE() << "the plant has no quaternion floating joint";
  return {};
}

// q for MakeFloatingBaseWorld(): identity quaternion, `p` for the base
// position, `elbow` for the joint.
VectorXd FloatingQ(const Vector3d& p, double elbow) {
  VectorXd q(8);
  q << 1.0, 0.0, 0.0, 0.0, p.x(), p.y(), p.z(), elbow;
  return q;
}

// ---------------------------------------------------------------------------
// 1. Joint scope: quaternion bases, and the constant-coordinate carve-out that
//    makes them usable anyway.
// ---------------------------------------------------------------------------

GTEST_TEST(ApiTest, MovingQuaternionBaseThrowsNamingTheJoint) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeFloatingBaseWorld();
  const auto checker = MakeCheckerPtr(model, SerialOptions());
  const std::string joint_name = FloatingJointName(model->plant());
  ASSERT_FALSE(joint_name.empty());

  // Move a quaternion coordinate: straight-line interpolation of quaternion
  // components is not a rotation-space geodesic, so the convex-hull motion
  // bound has no meaning and the library must refuse rather than guess. The
  // message must also point at the way out.
  Eigen::MatrixXd points(8, 2);
  points.col(0) = FloatingQ(Vector3d::Zero(), 0.0);
  points.col(1) = FloatingQ(Vector3d::Zero(), 0.0);
  points(0, 1) = 0.7071067811865476;  // w
  points(3, 1) = 0.7071067811865476;  // z
  EXPECT_THAT(ThrowMessage([&]() {
                checker->CheckTrajectory(BezierCurve<double>(0.0, 1.0, points));
              }),
              AllOf(HasSubstr(joint_name), HasSubstr("quaternion_floating"),
                    HasSubstr("constant-coordinate carve-out")));

  // Translating the base is refused for the same reason (the coordinate belongs
  // to an excluded joint), and the message names the coordinate index.
  Eigen::MatrixXd translated(8, 2);
  translated.col(0) = FloatingQ(Vector3d::Zero(), 0.0);
  translated.col(1) = FloatingQ(Vector3d(0.2, 0.0, 0.0), 0.0);
  EXPECT_THAT(
      ThrowMessage([&]() {
        checker->CheckTrajectory(BezierCurve<double>(0.0, 1.0, translated));
      }),
      AllOf(HasSubstr(joint_name), HasSubstr("coordinate 4")));
}

GTEST_TEST(ApiTest, ConstantQuaternionBaseIsAcceptedEndToEnd) {
  // A floating base whose pose is constant along the trajectory is treated as
  // welded, so a floating-base robot is usable as long as the trajectory does
  // not move the base. `wobble` is the sub-tolerance drift of the base's y
  // position: at exactly zero the carve-out is exact, while a base held only to
  // within the continuity tolerance is still carved but owes its residual to
  // the motion bound (motion_bound_test.cc proves that residual is charged).
  for (const double wobble : {0.0, 6e-8}) {
    SCOPED_TRACE("wobble = " + std::to_string(wobble));
    const auto checker =
        MakeCheckerPtr(MakeFloatingBaseWorld(), SerialOptions());
    Eigen::MatrixXd points(8, 3);
    for (int j = 0; j < 3; ++j) {
      points.col(j) = FloatingQ(Vector3d(0.05, -0.10, 0.0), 0.0);
    }
    points(5, 1) += wobble;
    points(7, 1) = 0.35;  // Only the elbow moves.
    points(7, 2) = 0.70;
    EXPECT_EQ(
        checker->CheckTrajectory(BezierCurve<double>(0.0, 1.0, points)).verdict,
        Verdict::kCertifiedFree);
  }
}

// ---------------------------------------------------------------------------
// 2. Geometry scope: rotating half spaces and deformables.
// ---------------------------------------------------------------------------

GTEST_TEST(ApiTest, RotatingHalfSpaceThrowsAtConstruction) {
  // A half space on a body that rotates relative to an unfiltered partner has
  // unbounded reach, so no finite lambda exists for that pair. This must be
  // refused when the checker is built, not discovered mid-certification, and
  // the message must say what to do about it ("Box").
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

  EXPECT_THAT(ThrowMessage([&]() {
                MakeCheckerPtr(model, SerialOptions());
              }),
              AllOf(HasSubstr("blade_halfspace"), HasSubstr("post_geom"),
                    HasSubstr("spin"), HasSubstr("Box")));
}

GTEST_TEST(ApiTest, AnchoredHalfSpaceUnderARotatingArmIsAccepted) {
  // The complement, so the rule above is not read as "half spaces are
  // unsupported". An anchored ground plane under a rotating arm is accepted,
  // because lambda then bounds the arm's points and signed distance is
  // symmetric.
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

  const auto checker = MakeCheckerPtr(builder.Build(), SerialOptions());
  EXPECT_EQ(
      checker->CheckEdge(VectorXd::Constant(1, 0.0), VectorXd::Constant(1, 1.5))
          .verdict,
      Verdict::kCertifiedFree);
}

GTEST_TEST(ApiTest, DeformableGeometryIsRefusedNamingIt) {
  // Deformables are out of scope: their motion is not described by the plant's
  // generalized positions, so no motion bound exists for them at all.
  // Registering one needs a discrete plant, which RobotDiagramBuilder's default
  // time step already gives, so the refusal is exercised on a real model.
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

  EXPECT_THAT(ThrowMessage([&]() {
                MakeCheckerPtr(model, SerialOptions());
              }),
              AllOf(HasSubstr("deformable"), HasSubstr("squishy_blob")));
}

// ---------------------------------------------------------------------------
// 3. Dimensions, trajectory validation and options.
// ---------------------------------------------------------------------------

GTEST_TEST(ApiTest, DimensionMismatchMessagesNameTheSizes) {
  std::shared_ptr<const RobotDiagram<double>> model = MakeArmWorld();
  const auto checker = MakeCheckerPtr(model, SerialOptions());
  ASSERT_EQ(model->plant().num_positions(), 2);

  EXPECT_THAT(ThrowMessage([&]() {
                checker->CheckPath(Eigen::MatrixXd::Zero(3, 4));
              }),
              AllOf(HasSubstr("CheckPath"), HasSubstr("3 rows"),
                    HasSubstr("2 generalized positions"),
                    HasSubstr("waypoints are columns")));

  EXPECT_THAT(ThrowMessage([&]() {
                checker->CheckEdge(VectorXd::Zero(2), VectorXd::Zero(5));
              }),
              AllOf(HasSubstr("CheckEdge"), HasSubstr("sizes 2 and 5")));

  EXPECT_THAT(ThrowMessage([&]() {
                checker->CheckTrajectory(
                    BezierCurve<double>(0.0, 1.0, Eigen::MatrixXd::Zero(7, 3)));
              }),
              AllOf(HasSubstr("7 rows"), HasSubstr("2 generalized positions")));

  // A single waypoint is not a path.
  DRAKE_EXPECT_THROWS_MESSAGE(checker->CheckPath(Eigen::MatrixXd::Zero(2, 1)),
                              ".*at least 2 waypoints.*");
}

GTEST_TEST(ApiTest, OptionsValidationMessagesAreActionable) {
  const auto checker = MakeCheckerPtr(MakeArmWorld(), SerialOptions());
  Eigen::MatrixXd points(2, 2);
  points << 0.0, 0.2, 0.0, 0.05;
  const BezierCurve<double> trajectory(0.0, 1.0, points);

  // Each case names the option the caller has to fix. A negative margin is in
  // the list because the displacement lemma is proved in the separated regime
  // only: an unreachable pair must be collision-filtered, not given a negative
  // threshold and silently "certified".
  struct Case {
    std::string needle;
    double Options::* field;
    double value;
  };
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  const double kInf = std::numeric_limits<double>::infinity();
  const std::vector<Case> cases = {
      {"distance_resolution", &Options::distance_resolution, 0.0},
      {"positive", &Options::distance_resolution, -1e-3},
      {"distance_resolution", &Options::distance_resolution, kNaN},
      {"finite", &Options::distance_resolution, kInf},
      {"nonnegative", &Options::margin, -0.01},
      {"margin", &Options::margin, kNaN},
  };
  for (const auto& [needle, field, value] : cases) {
    SCOPED_TRACE(needle);
    Options bad = SerialOptions();
    bad.*field = value;
    EXPECT_THAT(ThrowMessage([&]() {
                  checker->CheckTrajectory(trajectory, bad);
                }),
                HasSubstr(needle));
  }
}

GTEST_TEST(ApiTest, NullModelIsRefused) {
  // The adjacent finalization guard has no reachable input:
  // RobotDiagramBuilder::Build() finalizes unconditionally and RobotDiagram's
  // constructor is private to the builder. The null-model message names both
  // requirements, so this pins the wording for the pair.
  EXPECT_THAT(ThrowMessage([&]() {
                ContinuousCollisionChecker checker(nullptr);
              }),
              AllOf(HasSubstr("model is null"), HasSubstr("finalized")));
}

}  // namespace
}  // namespace continuous_collision
}  // namespace planning
}  // namespace drake
