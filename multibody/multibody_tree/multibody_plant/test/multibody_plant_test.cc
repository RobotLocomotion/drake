#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"

#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <tuple>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/math/transform.h"
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
using geometry::PenetrationAsPointPair;
using geometry::SceneGraph;
using math::RollPitchYaw;
using math::RotationMatrix;
using math::Transform;
using multibody::benchmarks::Acrobot;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::pendulum::MakePendulumPlant;
using multibody::benchmarks::pendulum::PendulumParameters;
using multibody::multibody_plant::MultibodyPlant;
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
using std::pair;
using std::make_pair;
using std::tie;
using std::unique_ptr;

namespace multibody {
namespace multibody_plant {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  template <typename T>
  static BodyIndex geometry_id_to_body_index(
      const MultibodyPlant<T>& plant, GeometryId id) {
    return plant.geometry_id_to_body_index_.at(id);
  }

  static MatrixX<double> CalcNormalSeparationVelocitiesJacobian(
      const MultibodyPlant<double>& plant, const Context<double>& context,
      const std::vector<PenetrationAsPointPair<double>>& point_pairs) {
    return plant.CalcNormalSeparationVelocitiesJacobian(context, point_pairs);
  }

  static MatrixX<double> CalcTangentVelocitiesJacobian(
      const MultibodyPlant<double>& plant, const Context<double>& context,
      const std::vector<PenetrationAsPointPair<double>>& point_pairs,
      std::vector<Matrix3<double>>* R_WC_set) {
    return plant.CalcTangentVelocitiesJacobian(context, point_pairs, R_WC_set);
  }
};

namespace {

// This test creates a simple model for an acrobot using MultibodyPlant and
// verifies a number of invariants such as that body and joint models were
// properly added and the model sizes.
GTEST_TEST(MultibodyPlant, SimpleModelCreation) {
  const std::string kInvalidName = "InvalidName";

  const AcrobotParameters parameters;
  unique_ptr<MultibodyPlant<double>> plant =
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
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/benchmarks/acrobot/acrobot.sdf");
    plant_ = builder.AddSystem<MultibodyPlant>();
    AddModelFromSdfFile(full_name, plant_, scene_graph_);
    // Add gravity to the model.
    plant_->AddForceElement<UniformGravityFieldElement>(
        -9.81 * Vector3<double>::UnitZ());
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
    plant_->Finalize(scene_graph_);

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

  void SetUpDiscreteAcrobotPlant(double time_step) {
    systems::DiagramBuilder<double> builder;
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/benchmarks/acrobot/acrobot.sdf");
    discrete_plant_ = std::make_unique<MultibodyPlant<double>>(time_step);
    AddModelFromSdfFile(full_name, discrete_plant_.get());
    // Add gravity to the model.
    discrete_plant_->AddForceElement<UniformGravityFieldElement>(
        -9.81 * Vector3<double>::UnitZ());
    discrete_plant_->Finalize();

    discrete_context_ = discrete_plant_->CreateDefaultContext();
    ASSERT_EQ(discrete_plant_->num_actuators(), 1);
    discrete_context_->FixInputPort(
        discrete_plant_->get_actuation_input_port().get_index(),
        Vector1<double>(0.0));
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

  // Verifies the computation performed by
  // MultibodyPlant::CalcDiscreteVariableUpdates(). For this simple model
  // without contact, we verify that this method performs the periodic update
  // of the state using a semi-explicit Euler strategy, that is:
  //   vⁿ⁺¹ = vⁿ + dt v̇ⁿ
  //   qⁿ⁺¹ = qⁿ + dt N(qⁿ) vⁿ⁺¹
  // To perform this verification, we compute v̇ⁿ using
  // MultibodyPlant::CalcTimeDerivatives().
  void VerifyDoCalcDiscreteVariableUpdates(double theta1, double theta2,
                                           double theta1dot, double theta2dot) {
    DRAKE_DEMAND(plant_ != nullptr);
    DRAKE_DEMAND(discrete_plant_ != nullptr);
    const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

    const double time_step = discrete_plant_->time_step();

    // Set the state for the continuous model:
    shoulder_->set_angle(context_.get(), theta1);
    elbow_->set_angle(context_.get(), theta2);
    shoulder_->set_angular_rate(context_.get(), theta1dot);
    elbow_->set_angular_rate(context_.get(), theta2dot);

    // Set the state for the discrete model:
    // Note: modeling elements such as joints, bodies, frames, etc. are agnostic
    // to whether the state is discrete or continuous. Therefore, we are allowed
    // to using the same modeling elements to set both `context` and
    // `discrete_context`.
    shoulder_->set_angle(discrete_context_.get(), theta1);
    elbow_->set_angle(discrete_context_.get(), theta2);
    shoulder_->set_angular_rate(discrete_context_.get(), theta1dot);
    elbow_->set_angular_rate(discrete_context_.get(), theta2dot);

    plant_->CalcTimeDerivatives(*context_, derivatives_.get());
    auto updates = discrete_plant_->AllocateDiscreteVariables();
    discrete_plant_->CalcDiscreteVariableUpdates(
        *discrete_context_, updates.get());

    // Copies to plain Eigen vectors to verify the math.
    const VectorXd x0 = context_->get_continuous_state_vector().CopyToVector();
    const VectorXd xdot = derivatives_->CopyToVector();
    const VectorXd xnext = updates->get_vector().CopyToVector();

    // Verify that xnext is updated using a semi-explicit strategy, that is:
    //   vnext = v0 + dt * vdot
    //   qnext = q0 + dt * vnext
    VectorXd xnext_expected(plant_->num_multibody_states());
    const int nv = plant_->num_velocities();
    const int nq = plant_->num_positions();
    xnext_expected.segment(nq, nv) =
        x0.segment(nq, nv) + time_step * xdot.segment(nq, nv);
    // We use the fact that nq = nv for this case.
    xnext_expected.segment(0, nq) =
        x0.segment(0, nq) + time_step * xnext_expected.segment(nq, nv);

    EXPECT_TRUE(CompareMatrices(
        xnext, xnext_expected, kTolerance, MatrixCompareType::relative));
  }

 protected:
  // The parameters of the model:
  const AcrobotParameters parameters_;
  // The model plant:
  MultibodyPlant<double>* plant_{nullptr};  // Owned by diagram_ below.
  std::unique_ptr<MultibodyPlant<double>> discrete_plant_;
  // A SceneGraph so that we can test geometry registration.
  SceneGraph<double>* scene_graph_{nullptr};
  // The Diagram containing both the MultibodyPlant and the SceneGraph.
  unique_ptr<Diagram<double>> diagram_;
  // Workspace including context and derivatives vector:
  unique_ptr<Context<double>> context_;
  unique_ptr<Context<double>> discrete_context_;
  unique_ptr<ContinuousState<double>> derivatives_;
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

// Verifies the correctness of MultibodyPlant::DoCalcDiscreteVariableUpdates()
// on a model of an acrobot.
TEST_F(AcrobotPlantTests, DoCalcDiscreteVariableUpdates) {
  // Set up an additional discrete state model of the same acrobot model.
  SetUpDiscreteAcrobotPlant(0.001 /* time step in seconds. */);

  // Verify the implementation for a number of arbitrarily chosen states.
  VerifyDoCalcDiscreteVariableUpdates(
      -M_PI / 5.0, M_PI / 2.0,  /* joint's angles */
      0.5, 1.0);                /* joint's angular rates */
  VerifyDoCalcDiscreteVariableUpdates(
      M_PI / 3.0, -M_PI / 5.0,  /* joint's angles */
      0.7, -1.0);               /* joint's angular rates */
  VerifyDoCalcDiscreteVariableUpdates(
      M_PI / 4.0, -M_PI / 3.0,  /* joint's angles */
      -0.5, 2.0);               /* joint's angular rates */
  VerifyDoCalcDiscreteVariableUpdates(
      -M_PI, -M_PI / 2.0,       /* joint's angles */
      -1.5, -2.5);              /* joint's angular rates */
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
  unique_ptr<systems::Context<double>> context =
      plant_->CreateDefaultContext();

  unique_ptr<AbstractValue> poses_value =
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
    // Also confirm the "maybe" variant works.
    const optional<FrameId> optional_id =
        plant_->GetBodyFrameIdIfExists(body_index);
    ASSERT_TRUE(optional_id.has_value());
    EXPECT_EQ(frame_id, *optional_id);
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

  // Similarly, the "optional" variant should return a null opt.
  optional<FrameId> undefined_id =
      plant_->GetBodyFrameIdIfExists(world_index());
  EXPECT_EQ(undefined_id, nullopt);
}

// Verifies that the right errors get invoked upon finalization.
GTEST_TEST(MultibodyPlantTest, FilterAdjacentBodiesSourceErrors) {
  SceneGraph<double> scene_graph;

  // Case: Finalize w/o having registered as geometry source but without
  // providing a scene graph -- no error.
  {
    MultibodyPlant<double> plant;
    EXPECT_NO_THROW(plant.Finalize());
  }

  // Case: Correct finalization -- registered as source and correct scene graph
  // provided -- no error.
  {
    MultibodyPlant<double> plant;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    EXPECT_NO_THROW(plant.Finalize(&scene_graph));
  }

  // Case: Registered as source, but no scene graph passed to Finalize() -
  // error.
  {
    MultibodyPlant<double> plant;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant.Finalize(), std::logic_error,
        "This MultibodyPlant has been registered as a SceneGraph geometry "
        "source. Finalize\\(\\) should be invoked with a pointer to the "
        "SceneGraph instance");
  }

  // Case: Registered as source, but *wrong* scene graph passed to Finalize() -
  // error.
  {
    MultibodyPlant<double> plant;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    SceneGraph<double> other_graph;
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant.Finalize(&other_graph), std::logic_error,
        "Finalizing on a SceneGraph instance must be performed on the SAME "
            "instance of SceneGraph used on the first call to "
            "RegisterAsSourceForSceneGraph\\(\\)");
  }

  // Case: Not registered as source, but passed SceneGraph in anyways.
  {
    MultibodyPlant<double> plant;
    EXPECT_NO_THROW(plant.Finalize(&scene_graph));
  }
}

// A class for constructing a multibody plant/scenegraph system consisting of
// a chain of spheres resting on the ground: S1 -> S2 -> S3 -> ... -> SN.
// In the zero configurations, the spheres are overlapping. However, due to
// adjacency filtering, no contact is reported between S1 & S2, S2 & S3, or,
// more, generally, between Si and Si+1. But there *should* be collisions
// between all other pairs: i.e., (S1, S3), (S1, S4), ..., (S1, SN), ...,
// (SN-1, SN) and between the ground and all elements: (G, S1), (G, S2), ...,
// (G, SN).
//
// The chain terminates with two additional bodies: one body registers a frame
// (but has no geometry), the other has no frame at all. It will have no bearing
// on collision tests but is used for geometry collection testing.
//
// Also accepts a filtering function that is applied between geometry
// registration and context compilation.
class SphereChainScenario {
 public:
  SphereChainScenario(
      int sphere_count,
      std::function<void(SphereChainScenario*)> apply_filters = nullptr) {
    using std::to_string;
    systems::DiagramBuilder<double> builder;
    scene_graph_ = builder.AddSystem<SceneGraph<double>>();
    plant_ = builder.AddSystem<MultibodyPlant<double>>();

    plant_->RegisterAsSourceForSceneGraph(scene_graph_);

    // A half-space for the ground geometry.
    ground_id_ = plant_->RegisterCollisionGeometry(
        plant_->world_body(),
        // A half-space passing through the origin in the x-z plane.
        geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero()),
        geometry::HalfSpace(), CoulombFriction<double>(), scene_graph_);

    auto make_sphere = [this](int i) {
      const double radius = 0.5;
      const RigidBody<double>& sphere = plant_->AddRigidBody(
          "Sphere" + to_string(i), SpatialInertia<double>());
      GeometryId sphere_id = plant_->RegisterCollisionGeometry(
          sphere, Isometry3d::Identity(), geometry::Sphere(radius),
          CoulombFriction<double>(), scene_graph_);
      // We add visual geometry to implicitly test that they are *not* included
      // in the collision results. We don't even save the ids for them.
      plant_->RegisterVisualGeometry(sphere, Isometry3d::Identity(),
                                     geometry::Sphere(radius), scene_graph_);
      return std::make_tuple(&sphere, sphere_id);
    };

    // Add sphere bodies.
    for (int i = 0; i < sphere_count; ++i) {
      // TODO(SeanCurtis-TRI): Make this prettier when C++17 is available.
      // E.g., auto [id, geometry] = make_sphere(i);
      GeometryId id{};
      const RigidBody<double>* body{};
      std::tie(body, id) = make_sphere(i);
      spheres_.push_back(body);
      sphere_ids_.push_back(id);
    }
    // Add hinges between spheres.
    for (int i = 0; i < sphere_count - 1; ++i) {
      plant_->AddJoint<RevoluteJoint>(
          "hinge" + to_string(i) + "_" + to_string(i + 1), *spheres_[i], {},
          *spheres_[i + 1], {}, Vector3d::UnitY());
    }

    // Body with no registered frame.
    no_geometry_body_ = &plant_->AddRigidBody("NothingRegistered",
                                              SpatialInertia<double>());

    // We are done defining the model.
    plant_->Finalize(scene_graph_);

    builder.Connect(
        plant_->get_geometry_poses_output_port(),
        scene_graph_->get_source_pose_port(*plant_->get_source_id()));
    builder.Connect(scene_graph_->get_query_output_port(),
                    plant_->get_geometry_query_input_port());

    if (apply_filters != nullptr) apply_filters(this);

    diagram_ = builder.Build();
    context_ = diagram_->CreateDefaultContext();

    // Set the zero configuration.
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, context_.get());

    // NOTE: Only ids for collision geometries are included.
    for (int i = 0; i < sphere_count; ++i) {
      unfiltered_collisions_.insert(std::make_pair(ground_id(), sphere_id(i)));
      for (int j = i + 2; j < sphere_count; ++j) {
        unfiltered_collisions_.insert(
            std::make_pair(sphere_id(i), sphere_id(j)));
      }
    }
  }

  // Perform the pair-wise collision detection.
  std::vector<geometry::PenetrationAsPointPair<double>>
  ComputePointPairPenetration() const {
    // Grab query object to test for collisions.
    const geometry::QueryObject<double>& query_object =
        plant_
            ->EvalAbstractInput(
                *plant_context_,
                plant_->get_geometry_query_input_port().get_index())
            ->GetValue<geometry::QueryObject<double>>();

    return query_object.ComputePointPairPenetration();
  }

  const RigidBody<double>& sphere(int i) const { return *spheres_.at(i); }
  const RigidBody<double>& no_geometry_body() const {
    return *no_geometry_body_;
  }
  const GeometryId sphere_id(int i) const { return sphere_ids_.at(i); }
  const GeometryId ground_id() const { return ground_id_; }
  MultibodyPlant<double>* mutable_plant() { return plant_; }
  SceneGraph<double>* mutable_scene_graph() { return scene_graph_; }

  // Reports all collisions that *should* be reported when no custom filters
  // have been applied.
  const std::set<std::pair<GeometryId, GeometryId>>& unfiltered_collisions()
      const {
    return unfiltered_collisions_;
  }

 private:
  // The diagram components.
  std::unique_ptr<Diagram<double>> diagram_{};
  std::unique_ptr<Context<double>> context_{};

  // Convenient handles into the diagram_ and context_.
  SceneGraph<double>* scene_graph_{};
  MultibodyPlant<double>* plant_{};
  systems::Context<double>* plant_context_{};

  // The bodies -- body Si is at index i.
  std::vector<const RigidBody<double>*> spheres_;
  std::vector<GeometryId> sphere_ids_;
  GeometryId ground_id_{};
  // The set of expected collision pairs with *no* user-defined collision
  // filters applied (but automatic filters -- adjacent bodies and geometries
  // affixed to the same body -- are accounted for).
  std::set<std::pair<GeometryId, GeometryId>> unfiltered_collisions_;
  const RigidBody<double>* no_geometry_body_{};
};

// Tests the automatic filtering of adjacent bodies.
// Introduces a ground plane with three spheres sitting on the plane.
// The spheres are linked in a chain: S1 -> S2 -> S3. In the zero configuration
// the spheres are overlapping. However, due to adjacency filtering, no
// contact is reported between S1 & S2, or S2 & S3. But there *should* be a
// collision between S1 & S3. Therefore, four total collisions will be reported:
// (G, S1), (G, S2), (G, S3), (S1, S3).
//
// NOTE: This *implicitly* tests that visual geometries are *not* included in
// collision because the `expected_pairs` only accounts for collision
// geometries.
GTEST_TEST(MultibodyPlantTest, FilterAdjacentBodies) {
  SphereChainScenario scenario(3);
  std::vector<geometry::PenetrationAsPointPair<double>> contacts =
      scenario.ComputePointPairPenetration();

  // The expected collisions.
  const std::set<std::pair<GeometryId, GeometryId>>& expected_pairs =
      scenario.unfiltered_collisions();
  ASSERT_EQ(contacts.size(), expected_pairs.size());

  auto expect_pair_in_set = [&expected_pairs](GeometryId id1, GeometryId id2) {
    auto pair1 = std::make_pair(id1, id2);
    auto pair2 = std::make_pair(id2, id1);
    if (expected_pairs.count(pair1) == 0 && expected_pairs.count(pair2) == 0) {
      GTEST_FAIL() << "The pair " << id1 << ", " << id2
                   << " is not in the expected set";
    }
  };
  for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
    const auto& point_pair = contacts[i];
    expect_pair_in_set(point_pair.id_A, point_pair.id_B);
  }
}

// Tests the error conditions for CollectRegisteredGeometries.
GTEST_TEST(MultibodyPlantTest, CollectRegisteredGeometriesErrors) {
  MultibodyPlant<double> plant;

  // A throw-away rigid body I can use to satisfy the function interface; it
  // will never be used because the function will fail in a pre-requisite test.
  RigidBody<double> body{SpatialInertia<double>()};
  // The case where the plant has *not* been finalized.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CollectRegisteredGeometries({&body}), std::logic_error,
      "Pre-finalize calls to 'CollectRegisteredGeometries\\(\\)' are not "
      "allowed; you must call Finalize\\(\\) first.");

  // The case where the plant has *not* been registered as a source.
  plant.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CollectRegisteredGeometries({&body}), std::runtime_error,
      "Failure .* in CollectRegisteredGeometries.* failed.");
}

// Tests the ability to accumulate the geometries associated with a set of
// bodies.
// NOTE: This exploits the implementation detail of GeometrySet that a FrameId
// is sufficient to capture one or more geometries. So, the contents of the
// set will be *typically* only be FrameIds -- specifically, the FrameId
// associated with each body -- even if no *geometries* are registered.
// The exception is if the world body is included, then anchored geometry ids
// will be included.
GTEST_TEST(MultibodyPlantTest, CollectRegisteredGeometries) {
  using geometry::GeometrySet;

  SphereChainScenario scenario(5);

  const MultibodyPlant<double>& plant = *scenario.mutable_plant();

  // Case: Empty vector produces empty geometry set.
  {
    GeometrySet set = plant.CollectRegisteredGeometries({});
    EXPECT_EQ(set.num_geometries(), 0);
    EXPECT_EQ(set.num_frames(), 0);
  }

  // Case: Single body produces single, corresponding frame.
  {
    GeometrySet set = plant.CollectRegisteredGeometries({&scenario.sphere(0)});
    EXPECT_EQ(set.num_geometries(), 0);
    EXPECT_EQ(set.num_frames(), 1);
    FrameId id_0 = plant.GetBodyFrameIdOrThrow(scenario.sphere(0).index());
    EXPECT_TRUE(set.contains(id_0));
  }

  // Case: Body with no corresponding geometry frame.
  {
    GeometrySet set =
        plant.CollectRegisteredGeometries({&scenario.no_geometry_body()});
    EXPECT_EQ(set.num_geometries(), 0);
    EXPECT_EQ(set.num_frames(), 0);
  }

  // Case: Include the world body.
  {
    GeometrySet set =
        plant.CollectRegisteredGeometries(
            {&scenario.mutable_plant()->world_body()});
    EXPECT_EQ(set.num_geometries(), 1);
    EXPECT_TRUE(set.contains(scenario.ground_id()));
    EXPECT_EQ(set.num_frames(), 0);
  }
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
  plant.Finalize(&scene_graph);

  EXPECT_EQ(plant.num_visual_geometries(), 0);
  EXPECT_EQ(plant.num_collision_geometries(), 3);
  EXPECT_TRUE(plant.geometry_source_is_registered());
  EXPECT_TRUE(plant.get_source_id());

  unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  // Place sphere 1 on top of the ground, with offset x = -x_offset.
  plant.model().SetFreeBodyPoseOrThrow(
      sphere1, Isometry3d(Translation3d(-x_offset, radius, 0.0)),
      context.get());
  // Place sphere 2 on top of the ground, with offset x = x_offset.
  plant.model().SetFreeBodyPoseOrThrow(
      sphere2, Isometry3d(Translation3d(x_offset, radius, 0.0)),
      context.get());

  unique_ptr<AbstractValue> poses_value =
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
  unique_ptr<MultibodyPlant<double>> pendulum =
      MakePendulumPlant(parameters);
  const auto& pin =
      pendulum->GetJointByName<RevoluteJoint>(parameters.pin_joint_name());
  unique_ptr<Context<double>> context = pendulum->CreateDefaultContext();
  context->FixInputPort(0, Vector1d{0.0});

  // First we will linearize about the unstable fixed point with the pendulum
  // in its inverted position.
  pin.set_angle(context.get(), M_PI);
  pin.set_angular_rate(context.get(), 0.0);

  unique_ptr<LinearSystem<double>> linearized_pendulum =
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
  unique_ptr<systems::Context<double>> context =
      plant_->CreateDefaultContext();

  // Set some non-zero state:
  shoulder_->set_angle(context.get(), M_PI / 3.0);
  elbow_->set_angle(context.get(), -0.2);
  shoulder_->set_angular_rate(context.get(), -0.5);
  elbow_->set_angular_rate(context.get(), 2.5);

  unique_ptr<AbstractValue> state_value =
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
  unique_ptr<Context<double>> context = plant.CreateDefaultContext();

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
  unique_ptr<Context<double>> context_;
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

// Verifies we can parse link collision geometries and surface friction.
GTEST_TEST(MultibodyPlantTest, ScalarConversionConstructor) {
  const std::string full_name = drake::FindResourceOrThrow(
      "drake/multibody/multibody_tree/parsing/test/"
          "links_with_visuals_and_collisions.sdf");
  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  AddModelFromSdfFile(full_name, &plant, &scene_graph);
  plant.Finalize(&scene_graph);

  EXPECT_EQ(plant.num_bodies(), 4);  // It includes the world body.
  EXPECT_EQ(plant.num_visual_geometries(), 5);
  EXPECT_EQ(plant.num_collision_geometries(), 3);

  const int link1_num_collisions =
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName("link1")).size();
  const int link2_num_collisions =
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName("link2")).size();
  const int link3_num_collisions =
      plant.GetCollisionGeometriesForBody(plant.GetBodyByName("link3")).size();
  ASSERT_EQ(link1_num_collisions, 2);
  ASSERT_EQ(link2_num_collisions, 0);
  ASSERT_EQ(link3_num_collisions, 1);

  const int link1_num_visuals =
      plant.GetVisualGeometriesForBody(plant.GetBodyByName("link1")).size();
  const int link2_num_visuals =
      plant.GetVisualGeometriesForBody(plant.GetBodyByName("link2")).size();
  const int link3_num_visuals =
      plant.GetVisualGeometriesForBody(plant.GetBodyByName("link3")).size();
  ASSERT_EQ(link1_num_visuals, 2);
  ASSERT_EQ(link2_num_visuals, 3);
  ASSERT_EQ(link3_num_visuals, 0);

  // Scalar convert the plant and verify invariants.
  MultibodyPlant<AutoDiffXd> plant_autodiff(plant);
  EXPECT_EQ(plant_autodiff.num_collision_geometries(),
            plant.num_collision_geometries());
  EXPECT_EQ(plant_autodiff.GetCollisionGeometriesForBody(
      plant_autodiff.GetBodyByName("link1")).size(), link1_num_collisions);
  EXPECT_EQ(plant_autodiff.GetCollisionGeometriesForBody(
      plant_autodiff.GetBodyByName("link2")).size(), link2_num_collisions);
  EXPECT_EQ(plant_autodiff.GetCollisionGeometriesForBody(
      plant_autodiff.GetBodyByName("link3")).size(), link3_num_collisions);
  EXPECT_EQ(plant_autodiff.GetVisualGeometriesForBody(
      plant_autodiff.GetBodyByName("link1")).size(), link1_num_visuals);
  EXPECT_EQ(plant_autodiff.GetVisualGeometriesForBody(
      plant_autodiff.GetBodyByName("link2")).size(), link2_num_visuals);
  EXPECT_EQ(plant_autodiff.GetVisualGeometriesForBody(
      plant_autodiff.GetBodyByName("link3")).size(), link3_num_visuals);
}

// This test is used to verify the correctness of the methods to compute the
// normal Jacobian N and the tangent Jacobian D.
// We do this for the particular case of a small box sitting on top of a larger
// box. The boxes interpenetrate by a small depth amount. A multicontact
// situation is emulated in which the contact engine supplied four point
// contact pairs corresponding to the four corners of the small box.
// To make the problem more interesting, and essentially to avoid considering
// a trivial case with zero translations and identity transformations, the
// entire setup with the small box on top of the large box is rotated about the
// z-axis by 45 degrees.
// We verify the correctness of the Jacobian matrices computed by
// MultibodyPlant by comparing them against a result obtained using automatic
// differentiation of the relative contact velocities.
class MultibodyPlantContactJacobianTests : public ::testing::Test {
 public:
  void SetUp() override {
    // Scene graph is only used to emulate a typical geometry registration.
    // Later we create the results of a query for the particular scenario in
    // this test by hand.
    plant_.RegisterAsSourceForSceneGraph(&scene_graph_);

    // The model simply contains a small and a large box.
    const RigidBody<double>& large_box =
        plant_.AddRigidBody("LargeBox", SpatialInertia<double>());
    large_box_id_ = plant_.RegisterCollisionGeometry(
        large_box, Isometry3d::Identity(),
        geometry::Box(large_box_size_, large_box_size_, large_box_size_),
        CoulombFriction<double>(), &scene_graph_);

    const RigidBody<double>& small_box =
        plant_.AddRigidBody("SmallBox", SpatialInertia<double>());
    small_box_id_ = plant_.RegisterCollisionGeometry(
        small_box, Isometry3d::Identity(),
        geometry::Box(small_box_size_, small_box_size_, small_box_size_),
        CoulombFriction<double>(), &scene_graph_);

    // We are done defining the model.
    plant_.Finalize(&scene_graph_);

    // Some sanity checks before proceeding.
    ASSERT_EQ(plant_.num_collision_geometries(), 2);
    ASSERT_TRUE(plant_.geometry_source_is_registered());
    ASSERT_TRUE(plant_.get_source_id());

    // Create the plant's context and set its state.
    context_ = plant_.CreateDefaultContext();
    SetBoxesOnSlantedConfiguration(context_.get());

    // Set the penetrations pairs consistent with the plant's state.
    SetPenetrationPairs(*context_, &penetrations_);
  }

  // Helper method to set the state of the system so that the small box sits
  // on top of the large box, with a small penetration depth.
  // The entire setup is rotated 45 degrees about the z axis into a slanted
  // configuration.
  void SetBoxesOnSlantedConfiguration(Context<double>* context) {
    // Notation for frames:
    //  - W: the world frame.
    //  - Lb: the frame of the large box, with its origin at the box's center.
    //  - Sb: the frame of the small box, with its origin at the box's center.

    const Body<double>& large_box = plant_.GetBodyByName("LargeBox");
    const Body<double>& small_box = plant_.GetBodyByName("SmallBox");

    const Transform<double> X_WLb =
        // Pure rotation.
        Transform<double>(RotationMatrix<double>::MakeZRotation(M_PI_4),
                          Vector3<double>::Zero()) *
        // Pure translation.
        Transform<double>(RotationMatrix<double>::Identity(),
                          Vector3<double>(0, -large_box_size_ / 2.0, 0));
    const Transform<double> X_WSb =
        // Pure rotation.
        Transform<double>(RotationMatrix<double>::MakeZRotation(M_PI_4),
                          Vector3<double>::Zero()) *
        // Pure translation.
        Transform<double>(RotationMatrix<double>::Identity(),
                          Vector3<double>(
                              0, small_box_size_ / 2.0 - penetration_, 0));

    plant_.model().SetFreeBodyPoseOrThrow(
        large_box, X_WLb.GetAsIsometry3(), context);
    plant_.model().SetFreeBodyPoseOrThrow(
        small_box, X_WSb.GetAsIsometry3(), context);
  }

  // Generate a valid set of penetrations for this particular setup that
  // emulates a multicontact scenario.
  void SetPenetrationPairs(
      const Context<double>& context,
      std::vector<PenetrationAsPointPair<double>>* penetrations) {
    const Body<double>& large_box = plant_.GetBodyByName("LargeBox");
    const Body<double>& small_box = plant_.GetBodyByName("SmallBox");

    std::vector<Isometry3<double>> X_WB_set;
    plant_.model().CalcAllBodyPosesInWorld(context, &X_WB_set);

    // Pose of the boxes in the world frame.
    const Isometry3<double>& X_WLb = X_WB_set[large_box.index()];
    const Isometry3<double>& X_WSb = X_WB_set[small_box.index()];

    // Normal pointing outwards from the top surface of the large box.
    const Vector3<double> nhat_large_box_W =
        X_WLb.rotation() * Vector3<double>::UnitY();

    for (double x : {-small_box_size_ / 2.0, small_box_size_ / 2.0}) {
      for (double z : {-small_box_size_ / 2.0, small_box_size_ / 2.0}) {
        PenetrationAsPointPair<double> point_pair;
        point_pair.id_A = large_box_id_;
        point_pair.id_B = small_box_id_;
        // Collision point on A (Large box).
        const Vector3<double> p_LbC(x, large_box_size_ / 2.0, z);
        point_pair.p_WCa = X_WLb * p_LbC;
        // Collision point on B (Small box).
        const Vector3<double> p_SbC(x, -small_box_size_ / 2.0, z);
        point_pair.p_WCb = X_WSb * p_SbC;
        point_pair.nhat_BA_W = -nhat_large_box_W;
        point_pair.depth = penetration_;
        penetrations->push_back(point_pair);
      }
    }
  }

  // Helper method to scalar convert the model and its context to AutoDiffXd.
  // The newly scalar converted context is set from the original context
  // templated on double such that we can take gradients with respect to the
  // generalized velocities.
  pair<unique_ptr<MultibodyPlant<AutoDiffXd>>,
       unique_ptr<Context<AutoDiffXd>>> ConvertPlantAndContextToAutoDiffXd() {
    // Scalar convert the plant and its context_.
    unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
        systems::System<double>::ToAutoDiffXd(plant_);
    unique_ptr<Context<AutoDiffXd>> context_autodiff =
        plant_autodiff->CreateDefaultContext();
    context_autodiff->SetTimeStateAndParametersFrom(*context_);

    // Initialize v_autodiff to have values v and so that it is the independent
    // variable of the problem.
    const VectorX<double> v =
        context_->get_continuous_state().get_generalized_velocity().
            CopyToVector();
    VectorX<AutoDiffXd> v_autodiff(plant_.num_velocities());
    math::initializeAutoDiff(v, v_autodiff);
    context_autodiff->get_mutable_continuous_state().
        get_mutable_generalized_velocity().SetFromVector(v_autodiff);

    return make_pair(std::move(plant_autodiff),
                          std::move(context_autodiff));
  }

  // Helper method to compute the separation velocity in the direction defined
  // by the normal nhat_BA for each contact pair in pairs_set. The i-th entry in
  // the output vector contains the separation velocity for the i-th pair in
  // pairs_set.
  // This method is templated to facilitate automatic differentiation for this
  // test.
  template <typename T>
  VectorX<T> CalcNormalVelocities(
      const MultibodyPlant<T>& plant_on_T,
      const Context<T>& context_on_T,
      const std::vector<PenetrationAsPointPair<double>>& pairs_set) const {
    std::vector<SpatialVelocity<T>> V_WB_set;
    plant_on_T.model().CalcAllBodySpatialVelocitiesInWorld(
        context_on_T, &V_WB_set);

    std::vector<Isometry3<T>> X_WB_set;
    plant_on_T.model().CalcAllBodyPosesInWorld(
        context_on_T, &X_WB_set);

    VectorX<T> vn(pairs_set.size());
    int icontact = 0;
    for (const auto& pair : pairs_set) {
      PenetrationAsPointPair<T> pair_on_T;
      BodyIndex bodyA_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_A);
      BodyIndex bodyB_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_B);
      const Vector3<T> p_WCa = pair.p_WCa.cast<T>();
      const Vector3<T> p_WAo = X_WB_set[bodyA_index].translation();
      const Vector3<T> p_AoCa_W = p_WCa - p_WAo;
      const SpatialVelocity<T> V_WA = V_WB_set[bodyA_index];
      const Vector3<T> v_WCa = V_WA.Shift(p_AoCa_W).translational();

      const Vector3<T> p_WCb = pair.p_WCb.cast<T>();
      const Vector3<T> p_WBo = X_WB_set[bodyB_index].translation();
      const Vector3<T> p_BoCb_W = p_WCb - p_WBo;
      const SpatialVelocity<T> V_WB = V_WB_set[bodyB_index];
      const Vector3<T> v_WCb = V_WB.Shift(p_BoCb_W).translational();

      // From the relative velocity of B in A, compute the normal separation
      // velocity vn (vn > 0 if bodies are moving apart)
      const Vector3<T> nhat_BA_W = pair.nhat_BA_W.cast<T>();
      vn(icontact++) = -nhat_BA_W.dot(v_WCb - v_WCa);
    }
    return vn;
  }

  // Helper method to compute the tangential velocities in a pair of directions
  // defined orthogonal to the normal nhat_BA for each contact pair in
  // pairs_set. Entries i and i+1 in the output vector contain the tangential
  // components of the relative velocity for the i-th pair in pairs_set.
  // This method is templated to facilitate automatic differentiation for this
  // test.
  template <typename T>
  VectorX<T> CalcTangentVelocities(
      const MultibodyPlant<T>& plant_on_T,
      const Context<T>& context_on_T,
      const std::vector<PenetrationAsPointPair<double>>& pairs_set,
      const std::vector<Matrix3<double>>& R_WC_set) const {
    std::vector<SpatialVelocity<T>> V_WB_set;
    plant_on_T.model().CalcAllBodySpatialVelocitiesInWorld(
        context_on_T, &V_WB_set);

    std::vector<Isometry3<T>> X_WB_set;
    plant_on_T.model().CalcAllBodyPosesInWorld(
        context_on_T, &X_WB_set);

    VectorX<T> vt(2 * pairs_set.size());
    int icontact = 0;
    for (const auto& pair : pairs_set) {
      PenetrationAsPointPair<T> pair_on_T;
      BodyIndex bodyA_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_A);
      BodyIndex bodyB_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_B);
      const Vector3<T> p_WCa = pair.p_WCa.cast<T>();
      const Vector3<T> p_WAo = X_WB_set[bodyA_index].translation();
      const Vector3<T> p_AoCa_W = p_WCa - p_WAo;
      const SpatialVelocity<T> V_WA = V_WB_set[bodyA_index];
      const Vector3<T> v_WCa = V_WA.Shift(p_AoCa_W).translational();

      const Vector3<T> p_WCb = pair.p_WCb.cast<T>();
      const Vector3<T> p_WBo = X_WB_set[bodyB_index].translation();
      const Vector3<T> p_BoCb_W = p_WCb - p_WBo;
      const SpatialVelocity<T> V_WB = V_WB_set[bodyB_index];
      const Vector3<T> v_WCb = V_WB.Shift(p_BoCb_W).translational();

      // The columns of R_WC (the orientation of contact frame C in the world),
      // contains the versors of C's basis, expressed in the world frame.
      // In particular, the first two columns corresponds to the versors tangent
      // to the contact plane.
      const Vector3<T> that1_W = R_WC_set[icontact].col(0).cast<T>();
      const Vector3<T> that2_W = R_WC_set[icontact].col(1).cast<T>();

      // Compute the relative velocity of B in A and obtain its components
      // in the contact frame C. The tangential velocities correspond to the
      // x and y components in this frame.
      vt(2 * icontact)     = that1_W.dot(v_WCb - v_WCa);
      vt(2 * icontact + 1) = that2_W.dot(v_WCb - v_WCa);

      icontact++;
    }
    return vt;
  }

 protected:
  MultibodyPlant<double> plant_;
  SceneGraph<double> scene_graph_;
  unique_ptr<Context<double>> context_;
  std::vector<PenetrationAsPointPair<double>> penetrations_;
  GeometryId small_box_id_;
  GeometryId large_box_id_;
  // Parameters of the setup.
  const double small_box_size_{1.0};
  const double large_box_size_{5.0};
  const double penetration_{0.01};
};

TEST_F(MultibodyPlantContactJacobianTests, NormalJacobian) {
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

  // Compute separation velocities Jacobian.
  const MatrixX<double> N =
      MultibodyPlantTester::CalcNormalSeparationVelocitiesJacobian(
          plant_, *context_, penetrations_);

  // Assert N has the right sizes.
  const int nv = plant_.num_velocities();
  const int nc = penetrations_.size();
  ASSERT_EQ(N.rows(), nc);
  ASSERT_EQ(N.cols(), nv);

  // Scalar convert the plant and its context_.
  unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff;
  unique_ptr<Context<AutoDiffXd>> context_autodiff;
  tie(plant_autodiff, context_autodiff) = ConvertPlantAndContextToAutoDiffXd();

  // Automatically differentiate vn (with respect to v) to get the normal
  // separation velocities Jacobian N.
  VectorX<AutoDiffXd> vn_autodiff = CalcNormalVelocities(
      *plant_autodiff, *context_autodiff, penetrations_);
  const MatrixX<double> vn_derivs =
      math::autoDiffToGradientMatrix(vn_autodiff);

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      N, vn_derivs, kTolerance, MatrixCompareType::relative));
}

TEST_F(MultibodyPlantContactJacobianTests, TangentJacobian) {
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

  // Store the orientation of the contact frames so that we can use them later
  // to compute the same Jacobian using autodifferentiation.
  std::vector<Matrix3<double>> R_WC_set;

  // Compute separation velocities Jacobian.
  const MatrixX<double> D =
      MultibodyPlantTester::CalcTangentVelocitiesJacobian(
          plant_, *context_, penetrations_, &R_WC_set);

  // Assert D has the right sizes.
  const int nv = plant_.num_velocities();
  const int nc = penetrations_.size();
  ASSERT_EQ(D.rows(), 2 * nc);
  ASSERT_EQ(D.cols(), nv);

  // Scalar convert the plant and its context_.
  unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff;
  unique_ptr<Context<AutoDiffXd>> context_autodiff;
  tie(plant_autodiff, context_autodiff) = ConvertPlantAndContextToAutoDiffXd();

  // Automatically differentiate vt (with respect to v) to get the tangent
  // velocities Jacobian D.
  VectorX<AutoDiffXd> vt_autodiff = CalcTangentVelocities(
      *plant_autodiff, *context_autodiff, penetrations_, R_WC_set);
  const MatrixX<double> vt_derivs =
      math::autoDiffToGradientMatrix(vt_autodiff);

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      D, vt_derivs, kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

