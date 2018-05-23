#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <functional>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {

using Eigen::AngleAxisd;
using Eigen::Isometry3d;
using Eigen::Matrix2d;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryId;
using geometry::SceneGraph;
using multibody::benchmarks::Acrobot;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::pendulum::MakePendulumPlant;
using multibody::benchmarks::pendulum::PendulumParameters;
using multibody::parsing::AddModelFromSdfFile;
using systems::AbstractValue;
using systems::BasicVector;
using systems::Context;
using systems::ContinuousState;
using systems::DiagramBuilder;
using systems::Diagram;
using systems::LinearSystem;
using systems::Linearize;
using systems::VectorBase;

namespace multibody {
namespace multibody_plant {
namespace {

// This test creates a simple model for an acrobot using MultibodyPlant and
// verifies a number of invariants such as that body and joint models were
// properly added and the model sizes.
GTEST_TEST(MultibodyPlant, SimpleModelCreation) {
  const std::string kInvalidName = "InvalidName";

  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeAcrobotPlant(parameters, true /* Make a finalized plant. */);

  // MakeAcrobotPlant() has already called Finalize() on the new acrobot plant.
  // Therefore attempting to call this method again will throw an exception.
  // Verify this.
  EXPECT_THROW(plant->Finalize(), std::logic_error);

  // Model Size. Counting the world body, there should be three bodies.
  EXPECT_EQ(plant->num_bodies(), 3);
  EXPECT_EQ(plant->num_joints(), 2);
  EXPECT_EQ(plant->num_actuators(), 1);
  EXPECT_EQ(plant->num_actuated_dofs(), 1);

  // State size.
  EXPECT_EQ(plant->num_positions(), 2);
  EXPECT_EQ(plant->num_velocities(), 2);
  EXPECT_EQ(plant->num_multibody_states(), 4);

  // Query if elements exist in the model.
  EXPECT_TRUE(plant->HasBodyNamed(parameters.link1_name()));
  EXPECT_TRUE(plant->HasBodyNamed(parameters.link2_name()));
  EXPECT_FALSE(plant->HasBodyNamed(kInvalidName));

  EXPECT_TRUE(plant->HasJointNamed(parameters.shoulder_joint_name()));
  EXPECT_TRUE(plant->HasJointNamed(parameters.elbow_joint_name()));
  EXPECT_FALSE(plant->HasJointNamed(kInvalidName));

  EXPECT_TRUE(plant->HasJointActuatorNamed(parameters.actuator_name()));
  EXPECT_FALSE(plant->HasJointActuatorNamed(kInvalidName));

  // Get links by name.
  const Body<double>& link1 = plant->GetBodyByName(parameters.link1_name());
  EXPECT_EQ(link1.name(), parameters.link1_name());
  const Body<double>& link2 = plant->GetBodyByName(parameters.link2_name());
  EXPECT_EQ(link2.name(), parameters.link2_name());

  // Attempting to retrieve a link that is not part of the model should throw
  // an exception.
  EXPECT_THROW(plant->GetBodyByName(kInvalidName), std::logic_error);

  // Get joints by name.
  const Joint<double>& shoulder_joint =
      plant->GetJointByName(parameters.shoulder_joint_name());
  EXPECT_EQ(shoulder_joint.name(), parameters.shoulder_joint_name());
  const Joint<double>& elbow_joint =
      plant->GetJointByName(parameters.elbow_joint_name());
  EXPECT_EQ(elbow_joint.name(), parameters.elbow_joint_name());
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // Templatized version to obtain retrieve a particular known type of joint.
  const RevoluteJoint<double>& shoulder =
      plant->GetJointByName<RevoluteJoint>(parameters.shoulder_joint_name());
  EXPECT_EQ(shoulder.name(), parameters.shoulder_joint_name());
  const RevoluteJoint<double>& elbow =
      plant->GetJointByName<RevoluteJoint>(parameters.elbow_joint_name());
  EXPECT_EQ(elbow.name(), parameters.elbow_joint_name());
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // MakeAcrobotPlant() has already called Finalize() on the acrobot model.
  // Therefore no more modeling elements can be added. Verify this.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->AddRigidBody("AnotherBody", SpatialInertia<double>()),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      "Post-finalize calls to '.*' are not allowed; "
      "calls to this method must happen before Finalize\\(\\).");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->AddJoint<RevoluteJoint>(
          "AnotherJoint", link1, {}, link2, {}, Vector3d::UnitZ()),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      "Post-finalize calls to '.*' are not allowed; "
      "calls to this method must happen before Finalize\\(\\).");
  // TODO(amcastro-tri): add test to verify that requesting a joint of the wrong
  // type throws an exception. We need another joint type to do so.
}

// Fixture to perform a number of computational tests on an acrobot model.
class AcrobotPlantTests : public ::testing::Test {
 public:
  // Creates MultibodyPlant for an acrobot model.
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    scene_graph_ = builder.AddSystem<SceneGraph>();
    // Make a non-finalized plant so that we can tests methods with pre/post
    // Finalize() conditions.
    plant_ =
        builder.AddSystem(MakeAcrobotPlant(parameters_, false, scene_graph_));
    // Sanity check on the availability of the optional source id before using
    // it.
    DRAKE_DEMAND(plant_->get_source_id() != nullopt);

    // Verify that methods with pre-Finalize() conditions throw accordingly.
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant_->get_geometry_poses_output_port(),
        std::logic_error,
        /* Verify this method is throwing for the right reasons. */
        "Pre-finalize calls to '.*' are not allowed; "
        "you must call Finalize\\(\\) first.");

    DRAKE_EXPECT_THROWS_MESSAGE(
        plant_->get_continuous_state_output_port(),
        std::logic_error,
        /* Verify this method is throwing for the right reasons. */
        "Pre-finalize calls to '.*' are not allowed; "
        "you must call Finalize\\(\\) first.");

    // Finalize() the plant before accessing its ports for communicating with
    // SceneGraph.
    plant_->Finalize();

    builder.Connect(
        plant_->get_geometry_poses_output_port(),
        scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
    // And build the Diagram:
    diagram_ = builder.Build();

    link1_ = &plant_->GetBodyByName(parameters_.link1_name());
    link2_ = &plant_->GetBodyByName(parameters_.link2_name());
    shoulder_ = &plant_->GetJointByName<RevoluteJoint>(
        parameters_.shoulder_joint_name());
    elbow_ = &plant_->GetJointByName<RevoluteJoint>(
        parameters_.elbow_joint_name());

    context_ = plant_->CreateDefaultContext();
    derivatives_ = plant_->AllocateTimeDerivatives();

    ASSERT_GT(plant_->num_actuators(), 0);
    input_port_ = &context_->FixInputPort(
        plant_->get_actuation_input_port().get_index(), Vector1<double>(0.0));
  }

  // Verifies the computation performed by MultibodyPlant::CalcTimeDerivatives()
  // for the acrobot model. The comparison is carried out against a benchmark
  // with hand written dynamics.
  void VerifyCalcTimeDerivatives(double theta1, double theta2,
                                 double theta1dot, double theta2dot,
                                 double input_torque) {
    const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

    // Set the state:
    shoulder_->set_angle(context_.get(), theta1);
    elbow_->set_angle(context_.get(), theta2);
    shoulder_->set_angular_rate(context_.get(), theta1dot);
    elbow_->set_angular_rate(context_.get(), theta2dot);

    // Fix input port to a value before computing anything. In this case, zero
    // actuation.
    input_port_->GetMutableVectorData<double>()->SetAtIndex(0, input_torque);

    plant_->CalcTimeDerivatives(*context_, derivatives_.get());
    const VectorXd xdot = derivatives_->CopyToVector();

    // Now compute inverse dynamics using our benchmark:
    Vector2d C_expected = acrobot_benchmark_.CalcCoriolisVector(
        theta1, theta2, theta1dot, theta2dot);
    Vector2d tau_g_expected =
        acrobot_benchmark_.CalcGravityVector(theta1, theta2);
    Vector2d rhs = tau_g_expected - C_expected + Vector2d(0.0, input_torque);
    Matrix2d M_expected = acrobot_benchmark_.CalcMassMatrix(theta2);
    Vector2d vdot_expected = M_expected.inverse() * rhs;
    VectorXd xdot_expected(4);
    xdot_expected << Vector2d(theta1dot, theta2dot), vdot_expected;

    EXPECT_TRUE(CompareMatrices(
        xdot, xdot_expected, kTolerance, MatrixCompareType::relative));
  }

 protected:
  // The parameters of the model:
  const AcrobotParameters parameters_;
  // The model plant:
  MultibodyPlant<double>* plant_{nullptr};
  // A SceneGraph so that we can test geometry registration.
  SceneGraph<double>* scene_graph_{nullptr};
  // The Diagram containing both the MultibodyPlant and the SceneGraph.
  std::unique_ptr<Diagram<double>> diagram_;
  // Workspace including context and derivatives vector:
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  // Non-owning pointers to the model's elements:
  const Body<double>* link1_{nullptr};
  const Body<double>* link2_{nullptr};
  const RevoluteJoint<double>* shoulder_{nullptr};
  const RevoluteJoint<double>* elbow_{nullptr};
  // Input port for the actuation:
  systems::FixedInputPortValue* input_port_{nullptr};

  // Reference benchmark for verification.
  Acrobot<double> acrobot_benchmark_{
      Vector3d::UnitZ() /* Plane normal */, Vector3d::UnitY() /* Up vector */,
      parameters_.m1(), parameters_.m2(),
      parameters_.l1(), parameters_.l2(),
      parameters_.lc1(), parameters_.lc2(),
      parameters_.Ic1(), parameters_.Ic2(),
      parameters_.b1(), parameters_.b2(),
      parameters_.g()};
};

// Verifies the correctness of MultibodyPlant::CalcTimeDerivatives() on a model
// of an acrobot.
TEST_F(AcrobotPlantTests, CalcTimeDerivatives) {
  // Some random tests with non-zero state:
  VerifyCalcTimeDerivatives(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0,                 /* joint's angular rates */
      -1.0);                    /* Actuation torque */
  VerifyCalcTimeDerivatives(
      M_PI / 3.0, -M_PI / 5.0,  /* joint's angles */
      0.7, -1.0,                /* joint's angular rates */
      1.0);                     /* Actuation torque */
  VerifyCalcTimeDerivatives(
      M_PI / 4.0, -M_PI / 3.0,  /* joint's angles */
      -0.5, 2.0,                /* joint's angular rates */
      -1.5);                    /* Actuation torque */
  VerifyCalcTimeDerivatives(
      -M_PI, -M_PI / 2.0,       /* joint's angles */
      -1.5, -2.5,               /* joint's angular rates */
      2.0);                     /* Actuation torque */
}

// Verifies the process of visual geometry registration with a SceneGraph
// for the acrobot model.
TEST_F(AcrobotPlantTests, VisualGeometryRegistration) {
  EXPECT_EQ(plant_->num_visual_geometries(), 3);
  EXPECT_TRUE(plant_->geometry_source_is_registered());
  EXPECT_TRUE(plant_->get_source_id());

  // The default context gets initialized by a call to SetDefaultState(), which
  // for a MultibodyPlant sets all revolute joints to have zero angles and zero
  // angular velocity.
  std::unique_ptr<systems::Context<double>> context =
      plant_->CreateDefaultContext();

  std::unique_ptr<AbstractValue> poses_value =
      plant_->get_geometry_poses_output_port().Allocate();
  EXPECT_NO_THROW(poses_value->GetValueOrThrow<FramePoseVector<double>>());
  const FramePoseVector<double>& poses =
      poses_value->GetValueOrThrow<FramePoseVector<double>>();
  EXPECT_EQ(poses.source_id(), plant_->get_source_id());
  EXPECT_EQ(poses.size(), 2);  // Only two frames move.

  // Compute the poses for each geometry in the model.
  plant_->get_geometry_poses_output_port().Calc(*context, poses_value.get());

  const MultibodyTree<double>& model = plant_->model();
  std::vector<Isometry3<double >> X_WB_all;
  model.CalcAllBodyPosesInWorld(*context, &X_WB_all);
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  for (BodyIndex body_index(1);
       body_index < plant_->num_bodies(); ++body_index) {
    const FrameId frame_id = plant_->GetBodyFrameIdOrThrow(body_index);
    const Isometry3<double>& X_WB = poses.value(frame_id);
    const Isometry3<double>& X_WB_expected = X_WB_all[body_index];
    EXPECT_TRUE(CompareMatrices(X_WB.matrix(), X_WB_expected.matrix(),
                                kTolerance, MatrixCompareType::relative));
  }

  // SceneGraph does not register a FrameId for the world. We use this fact
  // to test that GetBodyFrameIdOrThrow() throws an assertion for a body with no
  // FrameId, even though in this model we register an anchored geometry to the
  // world.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->GetBodyFrameIdOrThrow(world_index()),
      std::logic_error,
      /* Verify this method is throwing for the right reasons. */
      "Body 'WorldBody' does not have geometry registered with it.");
}

// Verifies the process of collision geometry registration with a
// SceneGraph.
// We build a model with two spheres and a ground plane. The ground plane is
// located at y = 0 with normal in the y-axis direction.
// For testing the output port computation we place the spheres on the ground
// plane with a given offset in the x direction from the origin.
GTEST_TEST(MultibodyPlantTest, CollisionGeometryRegistration) {
  // Parameters of the setup.
  const double radius = 0.5;
  const double x_offset = 0.6;

  SceneGraph<double> scene_graph;
  MultibodyPlant<double> plant;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // A half-space for the ground geometry.
  CoulombFriction<double> ground_friction(0.5, 0.3);
  GeometryId ground_id = plant.RegisterCollisionGeometry(
      plant.world_body(),
      // A half-space passing through the origin in the x-z plane.
      geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero()),
      geometry::HalfSpace(), ground_friction, &scene_graph);

  // Add two spherical bodies.
  const RigidBody<double>& sphere1 =
      plant.AddRigidBody("Sphere1", SpatialInertia<double>());
  CoulombFriction<double> sphere1_friction(0.8, 0.5);
  GeometryId sphere1_id = plant.RegisterCollisionGeometry(
      sphere1, Isometry3d::Identity(), geometry::Sphere(radius),
      sphere1_friction, &scene_graph);
  const RigidBody<double>& sphere2 =
      plant.AddRigidBody("Sphere2", SpatialInertia<double>());
  CoulombFriction<double> sphere2_friction(0.7, 0.6);
  GeometryId sphere2_id = plant.RegisterCollisionGeometry(
      sphere2, Isometry3d::Identity(), geometry::Sphere(radius),
      sphere2_friction, &scene_graph);

  // We are done defining the model.
  plant.Finalize();

  EXPECT_EQ(plant.num_visual_geometries(), 0);
  EXPECT_EQ(plant.get_num_collision_geometries(), 3);
  EXPECT_TRUE(plant.geometry_source_is_registered());
  EXPECT_TRUE(plant.get_source_id());

  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  // Place sphere 1 on top of the ground, with offset x = -x_offset.
  plant.model().SetFreeBodyPoseOrThrow(
      sphere1, Isometry3d(Translation3d(-x_offset, radius, 0.0)),
      context.get());
  // Place sphere 2 on top of the ground, with offset x = x_offset.
  plant.model().SetFreeBodyPoseOrThrow(
      sphere2, Isometry3d(Translation3d(x_offset, radius, 0.0)),
      context.get());

  std::unique_ptr<AbstractValue> poses_value =
      plant.get_geometry_poses_output_port().Allocate();
  EXPECT_NO_THROW(poses_value->GetValueOrThrow<FramePoseVector<double>>());
  const FramePoseVector<double>& pose_data =
      poses_value->GetValueOrThrow<FramePoseVector<double>>();
  EXPECT_EQ(pose_data.source_id(), plant.get_source_id());
  EXPECT_EQ(pose_data.size(), 2);  // Only two frames move.

  // Compute the poses for each geometry in the model.
  plant.get_geometry_poses_output_port().Calc(*context, poses_value.get());

  const MultibodyTree<double>& model = plant.model();
  std::vector<Isometry3<double >> X_WB_all;
  model.CalcAllBodyPosesInWorld(*context, &X_WB_all);
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  for (BodyIndex body_index(1);
       body_index < plant.num_bodies(); ++body_index) {
    const FrameId frame_id = plant.GetBodyFrameIdOrThrow(body_index);
    const Isometry3<double>& X_WB = pose_data.value(frame_id);
    const Isometry3<double>& X_WB_expected = X_WB_all[body_index];
    EXPECT_TRUE(CompareMatrices(X_WB.matrix(), X_WB_expected.matrix(),
                                kTolerance, MatrixCompareType::relative));
  }

  // Verify we can retrieve friction coefficients.
  EXPECT_TRUE(ExtractBoolOrThrow(
      plant.default_coulomb_friction(ground_id) == ground_friction));
  EXPECT_TRUE(ExtractBoolOrThrow(
      plant.default_coulomb_friction(sphere1_id) == sphere1_friction));
  EXPECT_TRUE(ExtractBoolOrThrow(
      plant.default_coulomb_friction(sphere2_id) == sphere2_friction));
}

GTEST_TEST(MultibodyPlantTest, LinearizePendulum) {
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

  PendulumParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> pendulum =
      MakePendulumPlant(parameters);
  const auto& pin =
      pendulum->GetJointByName<RevoluteJoint>(parameters.pin_joint_name());
  std::unique_ptr<Context<double>> context = pendulum->CreateDefaultContext();
  context->FixInputPort(0, Vector1d{0.0});

  // First we will linearize about the unstable fixed point with the pendulum
  // in its inverted position.
  pin.set_angle(context.get(), M_PI);
  pin.set_angular_rate(context.get(), 0.0);

  std::unique_ptr<LinearSystem<double>> linearized_pendulum =
      Linearize(*pendulum, *context,
                pendulum->get_actuation_input_port().get_index(),
                systems::kNoOutput);

  // Compute the expected solution by hand.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A <<                            0.0, 1.0,
      parameters.g() / parameters.l(), 0.0;
  B << 0, 1 / (parameters.m()* parameters.l() * parameters.l());
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->A(), A, kTolerance));
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->B(), B, kTolerance));

  // Now we linearize about the stable fixed point with the pendulum in its
  // downward position.
  pin.set_angle(context.get(), 0.0);
  pin.set_angular_rate(context.get(), 0.0);
  linearized_pendulum = Linearize(
      *pendulum, *context,
      pendulum->get_actuation_input_port().get_index(), systems::kNoOutput);
  // Compute the expected solution by hand.
  A <<                             0.0, 1.0,
      -parameters.g() / parameters.l(), 0.0;
  B << 0, 1 / (parameters.m()* parameters.l() * parameters.l());
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->A(), A, kTolerance));
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->B(), B, kTolerance));
}

TEST_F(AcrobotPlantTests, EvalContinuousStateOutputPort) {
  EXPECT_EQ(plant_->num_visual_geometries(), 3);
  EXPECT_TRUE(plant_->geometry_source_is_registered());
  EXPECT_TRUE(plant_->get_source_id());

  // The default context gets initialized by a call to SetDefaultState(), which
  // for a MultibodyPlant sets all revolute joints to have zero angles and zero
  // angular velocity.
  std::unique_ptr<systems::Context<double>> context =
      plant_->CreateDefaultContext();

  // Set some non-zero state:
  shoulder_->set_angle(context.get(), M_PI / 3.0);
  elbow_->set_angle(context.get(), -0.2);
  shoulder_->set_angular_rate(context.get(), -0.5);
  elbow_->set_angular_rate(context.get(), 2.5);

  std::unique_ptr<AbstractValue> state_value =
      plant_->get_continuous_state_output_port().Allocate();
  EXPECT_NO_THROW(state_value->GetValueOrThrow<BasicVector<double>>());
  const BasicVector<double>& state_out =
      state_value->GetValueOrThrow<BasicVector<double>>();
  EXPECT_EQ(state_out.size(), plant_->num_multibody_states());

  // Compute the poses for each geometry in the model.
  plant_->get_continuous_state_output_port().Calc(*context, state_value.get());

  // Get continuous state_out from context.
  const VectorBase<double>& state = context->get_continuous_state_vector();

  // Verify state_out indeed matches state.
  EXPECT_EQ(state_out.CopyToVector(), state.CopyToVector());
}

GTEST_TEST(MultibodyPlantTest, MapVelocityToQdotAndBack) {
  MultibodyPlant<double> plant;
  // This test is purely kinematic. Therefore we leave the spatial inertia
  // initialized to garbage. It should not affect the results.
  const RigidBody<double>& body =
      plant.AddRigidBody("FreeBody", SpatialInertia<double>());
  plant.Finalize();
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  // Set an arbitrary pose of the body in the world.
  const Vector3d p_WB(1, 2, 3);  // Position in world.
  const Vector3d axis_W =        // Orientation in world.
      (1.5 * Vector3d::UnitX() +
       2.0 * Vector3d::UnitY() +
       3.0 * Vector3d::UnitZ()).normalized();
  Isometry3d X_WB = Isometry3d::Identity();
  X_WB.linear() = AngleAxisd(M_PI / 3.0, axis_W).toRotationMatrix();
  X_WB.translation() = p_WB;
  plant.model().SetFreeBodyPoseOrThrow(body, X_WB, context.get());

  // Set an arbitrary, non-zero, spatial velocity of B in W.
  const SpatialVelocity<double> V_WB(Vector3d(1.0, 2.0, 3.0),
                                     Vector3d(-1.0, 4.0, -0.5));
  plant.model().SetFreeBodySpatialVelocityOrThrow(body, V_WB, context.get());

  // Use of MultibodyPlant's mapping to convert generalized velocities to time
  // derivatives of generalized coordinates.
  BasicVector<double> qdot(plant.num_positions());
  BasicVector<double> v(plant.num_velocities());
  ASSERT_EQ(qdot.size(), 7);
  ASSERT_EQ(v.size(), 6);
  v.SetFrom(context->get_continuous_state().get_generalized_velocity());
  plant.MapVelocityToQDot(*context, v, &qdot);

  // Mapping from qdot back to v should result in the original vector of
  // generalized velocities. Verify this.
  BasicVector<double> v_back(plant.num_velocities());
  plant.MapQDotToVelocity(*context, qdot, &v_back);

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(
      CompareMatrices(v_back.CopyToVector(), v.CopyToVector(), kTolerance));
}

// Test to verify we can still do dynamics even when there are weld joints
// within the model. This test builds a model from split_pendulum.sdf and
// therefore it must be kept in sync with that file. The model consists of a
// simple pendulum but built with two bodies and a WeldJoint joining them
// together into a single body. For details, refer to split_pendulum.sdf.
class SplitPendulum : public ::testing::Test {
 public:
  void SetUp() override {
    // Make the cart_pole model.
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/multibody_tree/"
        "multibody_plant/test/split_pendulum.sdf");
    AddModelFromSdfFile(full_name, &plant_);
    plant_.Finalize();

    // Get pin joint so that we can set the state.
    pin_ = &plant_.GetJointByName<RevoluteJoint>("pin");

    // Create a context to store the state for this model:
    context_ = plant_.CreateDefaultContext();
  }

 protected:
  MultibodyPlant<double> plant_;
  const RevoluteJoint<double>* pin_{nullptr};
  std::unique_ptr<Context<double>> context_;
};

// Verify the computation of the mass matrix against the analytical solution.
TEST_F(SplitPendulum, MassMatrix) {
  EXPECT_EQ(plant_.num_bodies(), 3);
  EXPECT_EQ(plant_.num_joints(), 2);
  EXPECT_EQ(plant_.num_positions(), 1);
  EXPECT_EQ(plant_.num_velocities(), 1);

  // Problem parameters. These must be kept in sync with split_pendulum.sdf.
  const double mass = 1.0;     // rod's mass.
  const double length = 12.0;  // rod's length.

  // Inertia of the entire rod of length 12.0 about the pivot point.
  const double Io = mass * length * length / 3.0;

  // We choose an arbitrary angle since the mass matrix is independent of the
  // state.
  const double theta = M_PI / 3;

  MatrixX<double> M(1, 1);
  pin_->set_angle(context_.get(), theta);
  plant_.model().CalcMassMatrixViaInverseDynamics(*context_, &M);

  // We can only expect values within the precision specified in the sdf file.
  EXPECT_NEAR(M(0, 0), Io, 1.0e-6);
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

