#include "drake/multibody/plant/multibody_plant.h"

#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <tuple>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/test_utilities/geometry_set_tester.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
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
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::benchmarks::Acrobot;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::pendulum::MakePendulumPlant;
using multibody::benchmarks::pendulum::PendulumParameters;
using multibody::Parser;
using systems::BasicVector;
using systems::ConstantVectorSource;
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

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  template <typename T>
  static BodyIndex geometry_id_to_body_index(
      const MultibodyPlant<T>& plant, GeometryId id) {
    return plant.geometry_id_to_body_index_.at(id);
  }

  static void CalcNormalAndTangentContactJacobians(
      const MultibodyPlant<double>& plant, const Context<double>& context,
      const std::vector<PenetrationAsPointPair<double>>& point_pairs,
      MatrixX<double>* Jn, MatrixX<double>* Jt,
      std::vector<RotationMatrix<double>>* R_WC_set) {
    plant.CalcNormalAndTangentContactJacobians(
        context, point_pairs, Jn, Jt, R_WC_set);
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
      MakeAcrobotPlant(parameters, false /* Don't make a finalized plant. */);

  // Model Size. Counting the world body, there should be three bodies.
  EXPECT_EQ(plant->num_bodies(), 3);
  EXPECT_EQ(plant->num_joints(), 2);
  EXPECT_EQ(plant->num_actuators(), 1);
  EXPECT_EQ(plant->num_actuated_dofs(), 1);

  // We expect to see the default and world model instances.
  EXPECT_EQ(plant->num_model_instances(), 2);

  // Add a split pendulum to the plant.
  const ModelInstanceIndex pendulum_model_instance =
      Parser(plant.get()).AddModelFromFile(FindResourceOrThrow(
          "drake/multibody/plant/test/split_pendulum.sdf"));
  EXPECT_EQ(plant->num_model_instances(), 3);

  plant->Finalize();
  // We should throw an exception if finalize is called twice.  Verify this.
  EXPECT_THROW(plant->Finalize(), std::logic_error);

  // Verify the final model size for the model as a whole and for each instance.
  EXPECT_EQ(plant->num_bodies(), 5);
  EXPECT_EQ(plant->num_joints(), 4);
  EXPECT_EQ(plant->num_actuators(), 2);
  EXPECT_EQ(plant->num_actuated_dofs(), 2);

  // World accessors.
  EXPECT_EQ(&plant->world_body(), &plant->world_body());
  EXPECT_EQ(&plant->world_frame(), &plant->world_frame());

  // State size.
  EXPECT_EQ(plant->num_positions(), 3);
  EXPECT_EQ(plant->num_velocities(), 3);
  EXPECT_EQ(plant->num_multibody_states(), 6);

  EXPECT_EQ(plant->num_actuated_dofs(default_model_instance()), 1);
  EXPECT_EQ(plant->num_positions(default_model_instance()), 2);
  EXPECT_EQ(plant->num_velocities(default_model_instance()), 2);

  EXPECT_EQ(plant->num_actuated_dofs(pendulum_model_instance), 1);
  EXPECT_EQ(plant->num_positions(pendulum_model_instance), 1);
  EXPECT_EQ(plant->num_velocities(pendulum_model_instance), 1);

  // Check that the input/output ports have the appropriate geometry.
  EXPECT_THROW(plant->get_actuation_input_port(), std::runtime_error);
  EXPECT_EQ(plant->get_actuation_input_port(
      default_model_instance()).size(), 1);
  EXPECT_EQ(plant->get_actuation_input_port(
      pendulum_model_instance).size(), 1);
  EXPECT_EQ(plant->get_continuous_state_output_port().size(), 6);
  EXPECT_EQ(plant->get_continuous_state_output_port(
      default_model_instance()).size(), 4);
  EXPECT_EQ(plant->get_continuous_state_output_port(
      pendulum_model_instance).size(), 2);

  // Check that model-instance ports get named properly.
  EXPECT_TRUE(plant->HasModelInstanceNamed("DefaultModelInstance"));
  EXPECT_TRUE(plant->HasModelInstanceNamed("SplitPendulum"));
  EXPECT_EQ(
      plant->get_actuation_input_port(default_model_instance()).get_name(),
      "DefaultModelInstance_actuation");
  EXPECT_EQ(plant->get_continuous_state_output_port(default_model_instance())
                .get_name(),
            "DefaultModelInstance_continuous_state");
  EXPECT_EQ(plant->get_continuous_state_output_port(pendulum_model_instance)
                .get_name(),
            "SplitPendulum_continuous_state");

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
  EXPECT_EQ(link1.model_instance(), default_model_instance());

  const Body<double>& link2 = plant->GetBodyByName(parameters.link2_name());
  EXPECT_EQ(link2.name(), parameters.link2_name());
  EXPECT_EQ(link2.model_instance(), default_model_instance());

  const Body<double>& upper = plant->GetBodyByName("upper_section");
  EXPECT_EQ(upper.model_instance(), pendulum_model_instance);

  const Body<double>& lower = plant->GetBodyByName("lower_section");
  EXPECT_EQ(lower.model_instance(), pendulum_model_instance);

  // Attempting to retrieve a link that is not part of the model should throw
  // an exception.
  EXPECT_THROW(plant->GetBodyByName(kInvalidName), std::logic_error);

  // Get body indices by model_instance.
  const std::vector<BodyIndex> acrobot_indices =
      plant->GetBodyIndices(default_model_instance());
  EXPECT_EQ(acrobot_indices.size(), 2);
  EXPECT_EQ(acrobot_indices[0], link1.index());
  EXPECT_EQ(acrobot_indices[1], link2.index());

  const std::vector<BodyIndex> pendulum_indices =
      plant->GetBodyIndices(pendulum_model_instance);
  EXPECT_EQ(pendulum_indices.size(), 2);
  EXPECT_EQ(pendulum_indices[0], upper.index());
  EXPECT_EQ(pendulum_indices[1], lower.index());

  // Get joints by name.
  const Joint<double>& shoulder_joint =
      plant->GetJointByName(parameters.shoulder_joint_name());
  EXPECT_EQ(shoulder_joint.name(), parameters.shoulder_joint_name());
  EXPECT_EQ(shoulder_joint.model_instance(), default_model_instance());
  Joint<double>& mutable_shoulder_joint =
      plant->GetMutableJointByName(parameters.shoulder_joint_name());
  EXPECT_EQ(&mutable_shoulder_joint, &shoulder_joint);
  const Joint<double>& elbow_joint =
      plant->GetJointByName(parameters.elbow_joint_name());
  EXPECT_EQ(elbow_joint.name(), parameters.elbow_joint_name());
  EXPECT_EQ(elbow_joint.model_instance(), default_model_instance());
  const Joint<double>& pin_joint =
      plant->GetJointByName("pin");
  EXPECT_EQ(pin_joint.model_instance(), pendulum_model_instance);
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // Get joint indices by model instance
  const std::vector<JointIndex> acrobot_joint_indices =
      plant->GetJointIndices(default_model_instance());
  EXPECT_EQ(acrobot_joint_indices.size(), 2);
  EXPECT_EQ(acrobot_joint_indices[0], shoulder_joint.index());
  EXPECT_EQ(acrobot_joint_indices[1], elbow_joint.index());

  const std::vector<JointIndex> pendulum_joint_indices =
      plant->GetJointIndices(pendulum_model_instance);
  EXPECT_EQ(pendulum_joint_indices.size(), 2);  // pin joint + weld joint.
  EXPECT_EQ(pendulum_joint_indices[0], pin_joint.index());

  // Templatized version to obtain retrieve a particular known type of joint.
  const RevoluteJoint<double>& shoulder =
      plant->GetJointByName<RevoluteJoint>(parameters.shoulder_joint_name());
  EXPECT_EQ(shoulder.name(), parameters.shoulder_joint_name());
  const RevoluteJoint<double>& elbow =
      plant->GetJointByName<RevoluteJoint>(parameters.elbow_joint_name());
  EXPECT_EQ(elbow.name(), parameters.elbow_joint_name());
  const RevoluteJoint<double>& pin =
      plant->GetJointByName<RevoluteJoint>("pin");
  EXPECT_EQ(pin.name(), "pin");
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // MakeAcrobotPlant() has already called Finalize() on the acrobot model.
  // Therefore no more modeling elements can be added. Verify this.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->AddRigidBody("AnotherBody", default_model_instance(),
                          SpatialInertia<double>()),
      std::logic_error,
      "Post-finalize calls to '.*' are not allowed; "
      "calls to this method must happen before Finalize\\(\\).");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->AddJoint<RevoluteJoint>(
          "AnotherJoint", link1, nullopt, link2, nullopt, Vector3d::UnitZ()),
      std::logic_error,
      "Post-finalize calls to '.*' are not allowed; "
      "calls to this method must happen before Finalize\\(\\).");
  // Test API for simplified `AddJoint` method.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->AddJoint(std::make_unique<RevoluteJoint<double>>(
          "AnotherJoint", link1.body_frame(), link2.body_frame(),
          Vector3d::UnitZ())),
      std::logic_error,
      ".*MultibodyTree.*finalized already.*");
  // TODO(amcastro-tri): add test to verify that requesting a joint of the wrong
  // type throws an exception. We need another joint type to do so.
}

GTEST_TEST(MultibodyPlantTest, AddMultibodyPlantSceneGraph) {
  systems::DiagramBuilder<double> builder;
  auto pair = AddMultibodyPlantSceneGraph(&builder);

  MultibodyPlant<double>* plant{};
  geometry::SceneGraph<double>* scene_graph{};
  // Check `tie` assignment.
  std::tie(plant, scene_graph) = pair;
  EXPECT_NE(plant, nullptr);
  EXPECT_NE(scene_graph, nullptr);
  // Check referencing.
  MultibodyPlant<double>& plant_ref = pair;
  EXPECT_EQ(&plant_ref, plant);

  // These should fail:
  // AddMultibodyPlantSceneGraphResult<double> extra(plant, scene_graph);
  // AddMultibodyPlantSceneGraphResult<double> extra{*plant, *scene_graph};
}

GTEST_TEST(ActuationPortsTest, CheckActuation) {
  // Create a MultibodyPlant consisting of two model instances, one actuated
  // and the other unactuated.
  MultibodyPlant<double> plant;
  const std::string acrobot_path = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/acrobot.sdf");
  const std::string cylinder_path = FindResourceOrThrow(
      "drake/multibody/benchmarks/free_body/uniform_solid_cylinder.urdf");
  auto acrobot_instance = Parser(&plant).AddModelFromFile(acrobot_path);
  auto cylinder_instance = Parser(&plant).AddModelFromFile(cylinder_path);
  plant.Finalize();

  // Verify the number of actuators.
  EXPECT_EQ(plant.num_actuated_dofs(acrobot_instance), 1);
  EXPECT_EQ(plant.num_actuated_dofs(cylinder_instance), 0);

  // Verify that we can get the actuation input ports.
  EXPECT_NO_THROW(plant.get_actuation_input_port());
  EXPECT_NO_THROW(plant.get_actuation_input_port(acrobot_instance));
  EXPECT_NO_THROW(plant.get_actuation_input_port(cylinder_instance));

  // Try to compute the derivatives without connecting the acrobot_instance
  // port.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  std::unique_ptr<ContinuousState<double>> continuous_state = plant.
      AllocateTimeDerivatives();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CalcTimeDerivatives(*context, continuous_state.get()),
      std::logic_error, "Actuation input port for model instance .* must "
          "be connected.");

  // Verify that derivatives can be computed after fixing the acrobot actuation
  // input port.
  context->FixInputPort(
      plant.get_actuation_input_port(acrobot_instance).get_index(),
      Vector1d(0.0));
  EXPECT_NO_THROW(plant.CalcTimeDerivatives(*context, continuous_state.get()));

  // Verify that derivatives can be computed after fixing the cylinder actuation
  // input port with an empty vector.
  context->FixInputPort(
      plant.get_actuation_input_port(cylinder_instance).get_index(),
      VectorXd(0));
  EXPECT_NO_THROW(plant.CalcTimeDerivatives(*context, continuous_state.get()));
}

// Fixture to perform a number of computational tests on an acrobot model.
class AcrobotPlantTests : public ::testing::Test {
 public:
  // Creates MultibodyPlant for an acrobot model.
  void SetUp() override {
    systems::DiagramBuilder<double> builder;
    // Make a non-finalized plant so that we can tests methods with pre/post
    // Finalize() conditions.
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/benchmarks/acrobot/acrobot.sdf");
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder);
    Parser(plant_).AddModelFromFile(full_name);
    // Add gravity to the model.
    plant_->AddForceElement<UniformGravityFieldElement>();
    // Sanity check on the availability of the optional source id before using
    // it.
    DRAKE_DEMAND(plant_->get_source_id() != nullopt);

    // Ensure that we can access the geometry ports pre-finalize.
    EXPECT_NO_THROW(plant_->get_geometry_query_input_port());
    EXPECT_NO_THROW(plant_->get_geometry_poses_output_port());

    DRAKE_EXPECT_THROWS_MESSAGE(
        plant_->get_continuous_state_output_port(),
        std::logic_error,
        /* Verify this method is throwing for the right reasons. */
        "Pre-finalize calls to '.*' are not allowed; "
        "you must call Finalize\\(\\) first.");

    // Finalize() the plant.
    plant_->Finalize();

    // And build the Diagram:
    diagram_ = builder.Build();

    link1_ = &plant_->GetBodyByName(parameters_.link1_name());
    link2_ = &plant_->GetBodyByName(parameters_.link2_name());
    shoulder_ = &plant_->GetMutableJointByName<RevoluteJoint>(
        parameters_.shoulder_joint_name());
    elbow_ = &plant_->GetMutableJointByName<RevoluteJoint>(
        parameters_.elbow_joint_name());

    context_ = diagram_->CreateDefaultContext();
    derivatives_ = diagram_->AllocateTimeDerivatives();
    plant_context_ = &diagram_->GetMutableSubsystemContext(
        *plant_, context_.get());

    ASSERT_GT(plant_->num_actuators(), 0);
    input_port_ = &plant_context_->FixInputPort(
        plant_->get_actuation_input_port().get_index(), Vector1<double>(0.0));
  }

  void SetUpDiscreteAcrobotPlant(double time_step) {
    systems::DiagramBuilder<double> builder;
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/benchmarks/acrobot/acrobot.sdf");
    discrete_plant_ = std::make_unique<MultibodyPlant<double>>(time_step);
    Parser(discrete_plant_.get()).AddModelFromFile(full_name);
    // Add gravity to the model.
    discrete_plant_->AddForceElement<UniformGravityFieldElement>();
    discrete_plant_->Finalize();

    discrete_context_ = discrete_plant_->CreateDefaultContext();
    ASSERT_EQ(discrete_plant_->num_actuators(), 1);
    discrete_context_->FixInputPort(
        discrete_plant_->get_actuation_input_port().get_index(),
        Vector1<double>(0.0));

    ASSERT_EQ(discrete_plant_->num_positions(), 2);
    ASSERT_EQ(discrete_plant_->num_velocities(), 2);
    ASSERT_EQ(discrete_plant_->GetPositions(*discrete_context_).size(), 2);
    ASSERT_EQ(discrete_plant_->GetVelocities(*discrete_context_).size(), 2);
  }

  // Computes the vector of generalized forces due to gravity.
  // This test is mostly to verify MultibodyPlant provides the proper APIs to
  // perform this computations.
  void VerifyCalcGravityGeneralizedForces(double theta1, double theta2) {
    const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

    // Set the state:
    shoulder_->set_angle(plant_context_, theta1);
    elbow_->set_angle(plant_context_, theta2);

    // Calculate the generalized forces due to gravity.
    const VectorX<double> tau_g =
        plant_->CalcGravityGeneralizedForces(*plant_context_);

    // Calculate a benchmark value.
    const Vector2d tau_g_expected =
        acrobot_benchmark_.CalcGravityVector(theta1, theta2);

    EXPECT_TRUE(CompareMatrices(
        tau_g, tau_g_expected, kTolerance, MatrixCompareType::relative));
  }

  // Verifies the computation performed by MultibodyPlant::CalcTimeDerivatives()
  // for the acrobot model. The comparison is carried out against a benchmark
  // with hand written dynamics.
  void VerifyCalcTimeDerivatives(double theta1, double theta2,
                                 double theta1dot, double theta2dot,
                                 double input_torque) {
    const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

    // Set the state:
    shoulder_->set_angle(plant_context_, theta1);
    elbow_->set_angle(plant_context_, theta2);
    shoulder_->set_angular_rate(plant_context_, theta1dot);
    elbow_->set_angular_rate(plant_context_, theta2dot);

    // Fix input port to a value before computing anything. In this case, zero
    // actuation.
    input_port_->GetMutableVectorData<double>()->SetAtIndex(0, input_torque);

    diagram_->CalcTimeDerivatives(*context_, derivatives_.get());
    const VectorXd xdot = derivatives_->CopyToVector();

    // Now compute inverse dynamics using our benchmark:
    const Vector2d C_expected = acrobot_benchmark_.CalcCoriolisVector(
        theta1, theta2, theta1dot, theta2dot);
    const Vector2d tau_g_expected =
        acrobot_benchmark_.CalcGravityVector(theta1, theta2);
    const Vector2d tau_damping(
        -parameters_.b1() * theta1dot, -parameters_.b2() * theta2dot);

    // Verify the computation of the contribution due to joint damping.
    MultibodyForces<double> forces(*plant_);
    shoulder_->AddInDamping(*plant_context_, &forces);
    elbow_->AddInDamping(*plant_context_, &forces);
    EXPECT_TRUE(CompareMatrices(forces.generalized_forces(), tau_damping,
                                kTolerance, MatrixCompareType::relative));

    // Verify the computation of xdot.
    const Vector2d rhs =
        tau_g_expected + tau_damping - C_expected + Vector2d(0.0, input_torque);
    const Matrix2d M_expected = acrobot_benchmark_.CalcMassMatrix(theta2);
    const Vector2d vdot_expected = M_expected.inverse() * rhs;
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
    shoulder_->set_angle(plant_context_, theta1);
    elbow_->set_angle(plant_context_, theta2);
    shoulder_->set_angular_rate(plant_context_, theta1dot);
    elbow_->set_angular_rate(plant_context_, theta2dot);

    // Set the state for the discrete model:
    const RevoluteJoint<double>& discrete_shoulder =
        discrete_plant_->GetJointByName<RevoluteJoint>(
            parameters_.shoulder_joint_name());
    const RevoluteJoint<double>& discrete_elbow =
        discrete_plant_->GetJointByName<RevoluteJoint>(
            parameters_.elbow_joint_name());
    discrete_shoulder.set_angle(discrete_context_.get(), theta1);
    discrete_elbow.set_angle(discrete_context_.get(), theta2);
    discrete_shoulder.set_angular_rate(discrete_context_.get(), theta1dot);
    discrete_elbow.set_angular_rate(discrete_context_.get(), theta2dot);

    diagram_->CalcTimeDerivatives(*context_, derivatives_.get());
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
  // Workspace including diagram context and derivatives vector:
  unique_ptr<Context<double>> context_;
  unique_ptr<Context<double>> discrete_context_;
  unique_ptr<ContinuousState<double>> derivatives_;
  // Non-owning pointer to the plant context.
  Context<double>* plant_context_{nullptr};
  // Non-owning pointers to the model's elements:
  const Body<double>* link1_{nullptr};
  const Body<double>* link2_{nullptr};
  RevoluteJoint<double>* shoulder_{nullptr};
  RevoluteJoint<double>* elbow_{nullptr};
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

// Verifies we can compute the vector of generalized forces due to gravity on a
// model of an acrobot.
TEST_F(AcrobotPlantTests, VerifyCalcGravityGeneralizedForces) {
  // Some arbitrary values of non-zero state:
  VerifyCalcGravityGeneralizedForces(
      -M_PI / 5.0, M_PI / 2.0  /* joint's angles */);
  VerifyCalcGravityGeneralizedForces(
      M_PI / 3.0, -M_PI / 5.0  /* joint's angles */);
  VerifyCalcGravityGeneralizedForces(
      M_PI / 4.0, -M_PI / 3.0  /* joint's angles */);
  VerifyCalcGravityGeneralizedForces(
      -M_PI, -M_PI / 2.0       /* joint's angles */);
}

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

  const ModelInstanceIndex instance_index =
      discrete_plant_->GetModelInstanceByName("acrobot");

  // The generalized contact forces output port should have the same size as
  // number of generalized velocities in the model instance, even if there is
  // no contact geometry in the model.
  EXPECT_EQ(discrete_plant_->get_generalized_contact_forces_output_port(
      instance_index).size(), 2);

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
  EXPECT_NO_THROW(poses_value->get_value<FramePoseVector<double>>());
  const FramePoseVector<double>& poses =
      poses_value->get_value<FramePoseVector<double>>();
  EXPECT_EQ(poses.source_id(), plant_->get_source_id());
  EXPECT_EQ(poses.size(), 2);  // Only two frames move.

  // Compute the poses for each geometry in the model.
  plant_->get_geometry_poses_output_port().Calc(*context, poses_value.get());

  const FrameId world_frame_id =
      plant_->GetBodyFrameIdOrThrow(plant_->world_body().index());
  ASSERT_TRUE(plant_->GetBodyFromFrameId(world_frame_id) != nullptr);
  EXPECT_EQ(plant_->GetBodyFromFrameId(world_frame_id)->index(),
            plant_->world_body().index());
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  for (BodyIndex body_index(1);
       body_index < plant_->num_bodies(); ++body_index) {
    const FrameId frame_id = plant_->GetBodyFrameIdOrThrow(body_index);
    // Also confirm the "maybe" variant works.
    const optional<FrameId> optional_id =
        plant_->GetBodyFrameIdIfExists(body_index);
    ASSERT_TRUE(optional_id.has_value());
    EXPECT_EQ(frame_id, *optional_id);
    EXPECT_EQ(body_index, plant_->GetBodyFromFrameId(frame_id)->index());
    const Isometry3<double>& X_WB = poses.value(frame_id);
    const Isometry3<double> X_WB_expected =
        plant_->EvalBodyPoseInWorld(*context, plant_->get_body(body_index));
    EXPECT_TRUE(CompareMatrices(X_WB.matrix(), X_WB_expected.matrix(),
                                kTolerance, MatrixCompareType::relative));
  }

  // TODO(SeanCurtis-TRI): These tests are no longer valid; there *is* a frame
  // id for the world body. This test needs to be changed so that there is
  // *another* body that doesn't have geometry.
#if 0
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
#endif
}

TEST_F(AcrobotPlantTests, SetDefaultState) {
  EXPECT_EQ(shoulder_->get_angle(*plant_context_), 0.0);
  EXPECT_EQ(elbow_->get_angle(*plant_context_), 0.0);

  // Set the default joint angles for the acrobot.
  shoulder_->set_default_angle(0.05);
  elbow_->set_default_angle(1.2);

  // New contexts should get the default angles.
  auto test_context = plant_->CreateDefaultContext();
  EXPECT_EQ(shoulder_->get_angle(*test_context), 0.05);
  EXPECT_EQ(elbow_->get_angle(*test_context), 1.2);

  shoulder_->set_default_angle(4.2);

  // Calling SetDefaultContext directly works, too.
  plant_->SetDefaultContext(plant_context_);
  EXPECT_EQ(shoulder_->get_angle(*plant_context_), 4.2);
}

TEST_F(AcrobotPlantTests, SetRandomState) {
  RandomGenerator generator;
  auto random_context = plant_->CreateDefaultContext();

  // Calling SetRandomContext before setting the distribution results in the
  // zero state.
  plant_->SetRandomContext(random_context.get(), &generator);
  EXPECT_TRUE(CompareMatrices(
      context_->get_mutable_continuous_state_vector().CopyToVector(),
      random_context->get_mutable_continuous_state_vector().CopyToVector()));

  // Setup distribution for random initial conditions.
  std::normal_distribution<symbolic::Expression> gaussian;
  shoulder_->set_random_angle_distribution(M_PI + 0.02*gaussian(generator));
  elbow_->set_random_angle_distribution(0.05*gaussian(generator));

  // This call should change the context.
  plant_->SetRandomContext(random_context.get(), &generator);
  EXPECT_FALSE(CompareMatrices(
      context_->get_mutable_continuous_state_vector().CopyToVector(),
      random_context->get_mutable_continuous_state_vector().CopyToVector()));

  const Eigen::VectorXd first_random_state =
      random_context->get_mutable_continuous_state_vector().CopyToVector();
  // And every call should return something different.
  plant_->SetRandomContext(random_context.get(), &generator);
  EXPECT_FALSE(CompareMatrices(
      first_random_state,
      random_context->get_mutable_continuous_state_vector().CopyToVector()));
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

  // Case: Registered as source, correct finalization.
  {
    MultibodyPlant<double> plant;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    EXPECT_NO_THROW(plant.Finalize());
  }

  // Case: Registered as source, but *wrong* scene graph passed to Finalize() -
  // error.
  {
    MultibodyPlant<double> plant;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    SceneGraph<double> other_graph;
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant.Finalize(&other_graph), std::logic_error,
        "Geometry registration.*first call to RegisterAsSourceForSceneGraph.*");
  }

  // Case: Not registered as source, but passed SceneGraph in anyways - error.
  {
    MultibodyPlant<double> plant;
    DRAKE_EXPECT_THROWS_MESSAGE(
        plant.Finalize(&scene_graph), std::logic_error,
        "This MultibodyPlant instance does not have a SceneGraph registered.*"
        "RegisterAsSourceForSceneGraph.*");
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
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder);

    // A half-space for the ground geometry.
    ground_id_ = plant_->RegisterCollisionGeometry(
        plant_->world_body(),
        // A half-space passing through the origin in the x-z plane.
        RigidTransformd(
            geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero())),
        geometry::HalfSpace(), "ground", CoulombFriction<double>());

    auto make_sphere = [this](int i) {
      const double radius = 0.5;
      const RigidBody<double>& sphere = plant_->AddRigidBody(
          "Sphere" + to_string(i), SpatialInertia<double>());
      GeometryId sphere_id = plant_->RegisterCollisionGeometry(
          sphere, RigidTransformd::Identity(), geometry::Sphere(radius),
          "collision", CoulombFriction<double>());
      // We add visual geometry to implicitly test that they are *not* included
      // in the collision results. We don't even save the ids for them.
      plant_->RegisterVisualGeometry(sphere, RigidTransformd::Identity(),
                                     geometry::Sphere(radius), "visual");
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
          "hinge" + to_string(i) + "_" + to_string(i + 1), *spheres_[i],
          nullopt, *spheres_[i + 1], nullopt, Vector3d::UnitY());
    }

    // Body with no registered frame.
    no_geometry_body_ = &plant_->AddRigidBody("NothingRegistered",
                                              SpatialInertia<double>());

    // We are done defining the model.
    plant_->Finalize();

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
    const auto& query_object = plant_->get_geometry_query_input_port().
        Eval<geometry::QueryObject<double>>(*plant_context_);
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
  using geometry::GeometrySetTester;

  SphereChainScenario scenario(5);

  const MultibodyPlant<double>& plant = *scenario.mutable_plant();

  // Case: Empty vector produces empty geometry set.
  {
    GeometrySet set = plant.CollectRegisteredGeometries({});
    GeometrySetTester tester(&set);
    EXPECT_EQ(tester.num_geometries(), 0);
    EXPECT_EQ(tester.num_frames(), 0);
  }

  // Case: Single body produces single, corresponding frame.
  {
    GeometrySet set = plant.CollectRegisteredGeometries({&scenario.sphere(0)});
    GeometrySetTester tester(&set);
    EXPECT_EQ(tester.num_geometries(), 0);
    EXPECT_EQ(tester.num_frames(), 1);
    FrameId id_0 = plant.GetBodyFrameIdOrThrow(scenario.sphere(0).index());
    EXPECT_TRUE(tester.contains(id_0));
  }

  // Case: Body with no corresponding geometry frame.
  {
    GeometrySet set =
        plant.CollectRegisteredGeometries({&scenario.no_geometry_body()});
    GeometrySetTester tester(&set);
    EXPECT_EQ(tester.num_geometries(), 0);
    EXPECT_EQ(tester.num_frames(), 0);
  }

  // Case: Include the world body.
  {
    GeometrySet set =
        plant.CollectRegisteredGeometries(
            {&scenario.mutable_plant()->world_body()});
    GeometrySetTester tester(&set);
    EXPECT_EQ(tester.num_frames(), 1);
    EXPECT_EQ(tester.num_geometries(), 0);
    EXPECT_FALSE(tester.contains(scenario.ground_id()));
  }
}

// Verifies the process of getting welded bodies.
GTEST_TEST(MultibodyPlantTest, GetBodiesWeldedTo) {
  using ::testing::UnorderedElementsAreArray;
  // This test expects that the following model has a world body and a pair of
  // welded-together bodies.
  const std::string sdf_file = FindResourceOrThrow(
      "drake/multibody/plant/test/split_pendulum.sdf");
  MultibodyPlant<double> plant;
  Parser(&plant).AddModelFromFile(sdf_file);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetBodiesWeldedTo(plant.world_body()), std::logic_error,
      "Pre-finalize calls to 'GetBodiesWeldedTo\\(\\)' are not "
      "allowed; you must call Finalize\\(\\) first.");
  plant.Finalize();
  const Body<double>& upper = plant.GetBodyByName("upper_section");
  const Body<double>& lower = plant.GetBodyByName("lower_section");
  EXPECT_THAT(
      plant.GetBodiesWeldedTo(plant.world_body()),
      UnorderedElementsAreArray({&plant.world_body()}));
  EXPECT_THAT(
      plant.GetBodiesWeldedTo(lower),
      UnorderedElementsAreArray({&upper, &lower}));
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
      RigidTransformd(
          geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero())),
      geometry::HalfSpace(), "ground", ground_friction);

  // Add two spherical bodies.
  const RigidBody<double>& sphere1 =
      plant.AddRigidBody("Sphere1", SpatialInertia<double>());
  CoulombFriction<double> sphere1_friction(0.8, 0.5);
  GeometryId sphere1_id = plant.RegisterCollisionGeometry(
      sphere1, RigidTransformd::Identity(), geometry::Sphere(radius),
      "collision", sphere1_friction);
  const RigidBody<double>& sphere2 =
      plant.AddRigidBody("Sphere2", SpatialInertia<double>());
  CoulombFriction<double> sphere2_friction(0.7, 0.6);
  GeometryId sphere2_id = plant.RegisterCollisionGeometry(
      sphere2, RigidTransformd::Identity(), geometry::Sphere(radius),
      "collision", sphere2_friction);

  // We are done defining the model.
  plant.Finalize();

  EXPECT_EQ(plant.num_visual_geometries(), 0);
  EXPECT_EQ(plant.num_collision_geometries(), 3);
  EXPECT_TRUE(plant.geometry_source_is_registered());
  EXPECT_TRUE(plant.get_source_id());

  unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  // Test the API taking a RigidTransform.
  auto X_WS1 = RigidTransformd(Vector3d(-x_offset, radius, 0.0));

  // Place sphere 1 on top of the ground, with offset x = -x_offset.
  plant.SetFreeBodyPose(
      context.get(), sphere1, X_WS1);
  // Place sphere 2 on top of the ground, with offset x = x_offset.
  plant.SetFreeBodyPose(
      context.get(), sphere2,
      Isometry3d(Translation3d(x_offset, radius, 0.0)));

  unique_ptr<AbstractValue> poses_value =
      plant.get_geometry_poses_output_port().Allocate();
  EXPECT_NO_THROW(poses_value->get_value<FramePoseVector<double>>());
  const FramePoseVector<double>& pose_data =
      poses_value->get_value<FramePoseVector<double>>();
  EXPECT_EQ(pose_data.source_id(), plant.get_source_id());
  EXPECT_EQ(pose_data.size(), 2);  // Only two frames move.

  // Compute the poses for each geometry in the model.
  plant.get_geometry_poses_output_port().Calc(*context, poses_value.get());

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  for (BodyIndex body_index(1);
       body_index < plant.num_bodies(); ++body_index) {
    const FrameId frame_id = plant.GetBodyFrameIdOrThrow(body_index);
    const Isometry3<double>& X_WB = pose_data.value(frame_id);
    const Isometry3<double> X_WB_expected =
        plant.EvalBodyPoseInWorld(*context, plant.get_body(body_index));
    EXPECT_TRUE(CompareMatrices(X_WB.matrix(), X_WB_expected.matrix(),
                                kTolerance, MatrixCompareType::relative));
  }

  // Verify we can retrieve friction coefficients.
  EXPECT_TRUE(
      plant.default_coulomb_friction(ground_id) == ground_friction);
  EXPECT_TRUE(
      plant.default_coulomb_friction(sphere1_id) == sphere1_friction);
  EXPECT_TRUE(
      plant.default_coulomb_friction(sphere2_id) == sphere2_friction);
}

// Verifies the process of visual geometry registration with a SceneGraph.
// We build a model with two spheres and a ground plane. The ground plane is
// located at y = 0 with normal in the y-axis direction.
GTEST_TEST(MultibodyPlantTest, VisualGeometryRegistration) {
  // Parameters of the setup.
  const double radius = 0.5;

  SceneGraph<double> scene_graph;
  MultibodyPlant<double> plant;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // A half-space for the ground geometry -- uses default visual material
  GeometryId ground_id = plant.RegisterVisualGeometry(
      plant.world_body(),
      // A half-space passing through the origin in the x-z plane.
      RigidTransformd(
          geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero())),
      geometry::HalfSpace(), "ground");

  // Add two spherical bodies.
  const RigidBody<double>& sphere1 =
      plant.AddRigidBody("Sphere1", SpatialInertia<double>());
  Vector4<double> sphere1_diffuse{0.9, 0.1, 0.1, 0.5};
  GeometryId sphere1_id = plant.RegisterVisualGeometry(
      sphere1, RigidTransformd::Identity(), geometry::Sphere(radius),
      "visual", sphere1_diffuse);
  const RigidBody<double>& sphere2 =
      plant.AddRigidBody("Sphere2", SpatialInertia<double>());
  Vector4<double> sphere2_diffuse{0.1, 0.9, 0.1, 0.5};
  GeometryId sphere2_id = plant.RegisterVisualGeometry(
      sphere2, RigidTransformd::Identity(), geometry::Sphere(radius),
      "visual", sphere2_diffuse);

  // We are done defining the model.
  plant.Finalize();

  EXPECT_EQ(plant.num_visual_geometries(), 3);
  EXPECT_EQ(plant.num_collision_geometries(), 0);
  EXPECT_TRUE(plant.geometry_source_is_registered());
  EXPECT_TRUE(plant.get_source_id());

  unique_ptr<Context<double>> context = scene_graph.CreateDefaultContext();
  unique_ptr<AbstractValue> state_value =
      scene_graph.get_query_output_port().Allocate();
  EXPECT_NO_THROW(state_value->get_value<QueryObject<double>>());
  const QueryObject<double>& query_object =
      state_value->get_value<QueryObject<double>>();
  scene_graph.get_query_output_port().Calc(*context, state_value.get());

  const SceneGraphInspector<double>& inspector = query_object.inspector();
  {
    const IllustrationProperties* material =
        inspector.GetIllustrationProperties(ground_id);
    ASSERT_NE(material, nullptr);
    // Undefined property value indicates use of default value.
    EXPECT_FALSE(material->HasProperty("phong", "diffuse"));
  }

  // This implicitly assumes that there *is* a "phong"|"diffuse" material. An
  // exception will be thrown otherwise.
  auto get_diffuse_color = [&inspector](GeometryId id) -> Vector4<double> {
    const IllustrationProperties* material =
        inspector.GetIllustrationProperties(id);
    return
        material->GetProperty<Vector4<double>>("phong", "diffuse");
  };
  {
    const Vector4<double>& test_diffuse = get_diffuse_color(sphere1_id);
    EXPECT_TRUE(CompareMatrices(test_diffuse, sphere1_diffuse, 0.0,
                                MatrixCompareType::absolute));
  }

  {
    const Vector4<double>& test_diffuse = get_diffuse_color(sphere2_id);
    EXPECT_TRUE(CompareMatrices(test_diffuse, sphere2_diffuse, 0.0,
                                MatrixCompareType::absolute));
  }
}

GTEST_TEST(MultibodyPlantTest, LinearizePendulum) {
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

  PendulumParameters parameters;
  unique_ptr<MultibodyPlant<double>> pendulum = MakePendulumPlant(parameters);
  const auto& pin =
      pendulum->GetJointByName<RevoluteJoint>(parameters.pin_joint_name());
  unique_ptr<Context<double>> context = pendulum->CreateDefaultContext();
  context->FixInputPort(pendulum->get_actuation_input_port().get_index(),
                        Vector1d{0.0});
  context->FixInputPort(
      pendulum->get_applied_generalized_force_input_port().get_index(),
      Vector1d{0.0});

  // First we will linearize about the unstable fixed point with the pendulum
  // in its inverted position.
  pin.set_angle(context.get(), M_PI);
  pin.set_angular_rate(context.get(), 0.0);

  unique_ptr<LinearSystem<double>> linearized_pendulum =
      Linearize(*pendulum, *context,
                pendulum->get_actuation_input_port().get_index(),
                systems::OutputPortSelection::kNoOutput);

  // Compute the expected solution by hand.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  const double domegadot_domega = -parameters.damping() /
      (parameters.m() * parameters.l() * parameters.l());
  A << 0.0, 1.0,
       parameters.g() / parameters.l(), domegadot_domega;
  B << 0, 1 / (parameters.m() * parameters.l() * parameters.l());
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->A(), A, kTolerance));
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->B(), B, kTolerance));

  // Now we linearize about the stable fixed point with the pendulum in its
  // downward position.
  pin.set_angle(context.get(), 0.0);
  pin.set_angular_rate(context.get(), 0.0);
  linearized_pendulum = Linearize(
      *pendulum, *context,
      pendulum->get_actuation_input_port().get_index(),
      systems::OutputPortSelection::kNoOutput);
  // Compute the expected solution by hand.
  A << 0.0, 1.0,
      -parameters.g() / parameters.l(), domegadot_domega;
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
  EXPECT_NO_THROW(state_value->get_value<BasicVector<double>>());
  const BasicVector<double>& state_out =
      state_value->get_value<BasicVector<double>>();
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
  const math::RigidTransformd X_WB(AngleAxisd(M_PI / 3.0, axis_W), p_WB);
  plant.SetFreeBodyPose(
      context.get(), body, X_WB.GetAsIsometry3());

  // Set an arbitrary, non-zero, spatial velocity of B in W.
  const SpatialVelocity<double> V_WB(Vector3d(1.0, 2.0, 3.0),
                                     Vector3d(-1.0, 4.0, -0.5));
  plant.SetFreeBodySpatialVelocity(context.get(), body, V_WB);

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
        "drake/multibody/plant/test/split_pendulum.sdf");
    Parser(&plant_).AddModelFromFile(full_name);
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
  plant_.CalcMassMatrixViaInverseDynamics(*context_, &M);

  // We can only expect values within the precision specified in the sdf file.
  EXPECT_NEAR(M(0, 0), Io, 1.0e-6);
}

// Verifies we can parse link collision geometries and surface friction.
GTEST_TEST(MultibodyPlantTest, ScalarConversionConstructor) {
  const std::string full_name = drake::FindResourceOrThrow(
      "drake/multibody/parsing/test/"
          "links_with_visuals_and_collisions.sdf");
  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  Parser(&plant, &scene_graph).AddModelFromFile(full_name);

  // Try scalar-converting pre-finalize - error.
  // N.B. Use extra parentheses; otherwise, compiler may think this is a
  // declaration.
  DRAKE_EXPECT_THROWS_MESSAGE(
      (MultibodyPlant<AutoDiffXd>(plant)), std::logic_error,
      ".*MultibodyTree with an invalid topology.*");

  plant.Finalize();

  EXPECT_EQ(plant.num_bodies(), 4);  // It includes the world body.
  EXPECT_TRUE(plant.geometry_source_is_registered());
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
  EXPECT_TRUE(plant_autodiff.geometry_source_is_registered());
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
  for (const auto& link_name : {"link1", "link2", "link3"}) {
    auto collision_geometries = plant_autodiff.GetCollisionGeometriesForBody(
        plant_autodiff.GetBodyByName(link_name));
    for (const auto& geometry : collision_geometries) {
      EXPECT_EQ(plant_autodiff.default_coulomb_friction(geometry),
                plant.default_coulomb_friction(geometry));
    }
  }

  // Make sure the geometry ports were included in the autodiffed plant.
  EXPECT_NO_THROW(plant_autodiff.get_geometry_query_input_port());
  EXPECT_NO_THROW(plant_autodiff.get_geometry_poses_output_port());
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
        large_box, RigidTransformd::Identity(),
        geometry::Box(large_box_size_, large_box_size_, large_box_size_),
        "collision", CoulombFriction<double>());

    const RigidBody<double>& small_box =
        plant_.AddRigidBody("SmallBox", SpatialInertia<double>());
    small_box_id_ = plant_.RegisterCollisionGeometry(
        small_box, RigidTransformd::Identity(),
        geometry::Box(small_box_size_, small_box_size_, small_box_size_),
        "collision", CoulombFriction<double>());

    // We are done defining the model.
    plant_.Finalize();

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

    const RigidTransform<double> X_WLb =
        // Pure rotation.
        RigidTransform<double>(RotationMatrix<double>::MakeZRotation(M_PI_4),
                               Vector3<double>::Zero()) *
        // Pure translation.
        RigidTransform<double>(RotationMatrix<double>::Identity(),
                               Vector3<double>(0, -large_box_size_ / 2.0, 0));
    const RigidTransform<double> X_WSb =
        // Pure rotation.
        RigidTransform<double>(RotationMatrix<double>::MakeZRotation(M_PI_4),
                               Vector3<double>::Zero()) *
        // Pure translation.
        RigidTransform<double>(RotationMatrix<double>::Identity(),
               Vector3<double>(0, small_box_size_ / 2.0 - penetration_, 0));

    plant_.SetFreeBodyPose(
        context, large_box, X_WLb.GetAsIsometry3());
    plant_.SetFreeBodyPose(
        context, small_box, X_WSb.GetAsIsometry3());
  }

  // Generate a valid set of penetrations for this particular setup that
  // emulates a multicontact scenario.
  void SetPenetrationPairs(
      const Context<double>& context,
      std::vector<PenetrationAsPointPair<double>>* penetrations) {
    const Body<double>& large_box = plant_.GetBodyByName("LargeBox");
    const Body<double>& small_box = plant_.GetBodyByName("SmallBox");

    // Pose of the boxes in the world frame.
    const Isometry3<double>& X_WLb =
        plant_.EvalBodyPoseInWorld(context, large_box);
    const Isometry3<double>& X_WSb =
        plant_.EvalBodyPoseInWorld(context, small_box);

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
    VectorX<T> vn(pairs_set.size());
    int icontact = 0;
    for (const auto& pair : pairs_set) {
      PenetrationAsPointPair<T> pair_on_T;

      BodyIndex bodyA_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_A);
      const Isometry3<T> X_WA = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyA_index));
      const SpatialVelocity<T> V_WA =
          plant_on_T.EvalBodySpatialVelocityInWorld(
              context_on_T, plant_on_T.get_body(bodyA_index));

      BodyIndex bodyB_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_B);
      const Isometry3<T> X_WB = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyB_index));
      const SpatialVelocity<T> V_WB =
          plant_on_T.EvalBodySpatialVelocityInWorld(
              context_on_T, plant_on_T.get_body(bodyB_index));

      const Vector3<T> p_WCa = pair.p_WCa.cast<T>();

      const Vector3<T> p_WAo = X_WA.translation();
      const Vector3<T> p_AoCa_W = p_WCa - p_WAo;
      const Vector3<T> v_WCa = V_WA.Shift(p_AoCa_W).translational();

      const Vector3<T> p_WCb = pair.p_WCb.cast<T>();
      const Vector3<T> p_WBo = X_WB.translation();
      const Vector3<T> p_BoCb_W = p_WCb - p_WBo;
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
      const std::vector<RotationMatrix<double>>& R_WC_set) const {
    VectorX<T> vt(2 * pairs_set.size());
    int icontact = 0;
    for (const auto& pair : pairs_set) {
      PenetrationAsPointPair<T> pair_on_T;

      BodyIndex bodyA_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_A);
      const Isometry3<T> X_WA = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyA_index));
      const SpatialVelocity<T> V_WA =
          plant_on_T.EvalBodySpatialVelocityInWorld(
              context_on_T, plant_on_T.get_body(bodyA_index));

      BodyIndex bodyB_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_B);
      const Isometry3<T> X_WB = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyB_index));
      const SpatialVelocity<T> V_WB =
          plant_on_T.EvalBodySpatialVelocityInWorld(
              context_on_T, plant_on_T.get_body(bodyB_index));

      const Vector3<T> p_WCa = pair.p_WCa.cast<T>();
      const Vector3<T> p_WAo = X_WA.translation();
      const Vector3<T> p_AoCa_W = p_WCa - p_WAo;
      const Vector3<T> v_WCa = V_WA.Shift(p_AoCa_W).translational();

      const Vector3<T> p_WCb = pair.p_WCb.cast<T>();
      const Vector3<T> p_WBo = X_WB.translation();
      const Vector3<T> p_BoCb_W = p_WCb - p_WBo;
      const Vector3<T> v_WCb = V_WB.Shift(p_BoCb_W).translational();

      // The columns of R_WC (the orientation of contact frame C in the world),
      // contains the versors of C's basis, expressed in the world frame.
      // In particular, the first two columns corresponds to the versors tangent
      // to the contact plane.
      const Vector3<T> that1_W = R_WC_set[icontact].matrix().col(0).cast<T>();
      const Vector3<T> that2_W = R_WC_set[icontact].matrix().col(1).cast<T>();

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

TEST_F(MultibodyPlantContactJacobianTests, NormalAndTangentJacobian) {
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

  // Store the orientation of the contact frames so that we can use them later
  // to compute the same Jacobian using autodifferentiation.
  std::vector<RotationMatrix<double>> R_WC_set;

  // Compute separation velocities Jacobian.
  MatrixX<double> N, D;
  MultibodyPlantTester::CalcNormalAndTangentContactJacobians(
          plant_, *context_, penetrations_, &N, &D, &R_WC_set);

  // Assert Jt has the right sizes.
  const int nv = plant_.num_velocities();
  const int nc = penetrations_.size();

  ASSERT_EQ(N.rows(), nc);
  ASSERT_EQ(N.cols(), nv);

  ASSERT_EQ(D.rows(), 2 * nc);
  ASSERT_EQ(D.cols(), nv);

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

  // Automatically differentiate vt (with respect to v) to get the tangent
  // velocities Jacobian Jt.
  VectorX<AutoDiffXd> vt_autodiff = CalcTangentVelocities(
      *plant_autodiff, *context_autodiff, penetrations_, R_WC_set);
  const MatrixX<double> vt_derivs =
      math::autoDiffToGradientMatrix(vt_autodiff);

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      D, vt_derivs, kTolerance, MatrixCompareType::relative));
}

// Verifies that we can obtain the indexes into the state vector for each joint
// in the model of a Kuka arm.
// For this topologically simple model with only one branch of bodies with root
// in the world, joints, and their degrees of freedom, are numbered from root
// (world) in increasing order towards the end effector.
GTEST_TEST(KukaModel, JointIndexes) {
  const char kSdfPath[] =
      "drake/manipulation/models/iiwa_description/sdf/"
          "iiwa14_no_collision.sdf";

  MultibodyPlant<double> plant;
  Parser(&plant).AddModelFromFile(FindResourceOrThrow(kSdfPath));
  const auto& base_link_frame = plant.GetFrameByName("iiwa_link_0");
  const Joint<double>& weld = plant.WeldFrames(
      plant.world_frame(), base_link_frame);
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 7);
  EXPECT_EQ(plant.num_velocities(), 7);

  // We expect the last joint to be the one WeldJoint fixing the model to the
  // world, since we added it last above with the call to WeldFrames().
  // We verify this assumption.
  ASSERT_EQ(weld.index(), plant.num_joints() - 1);

  // Verify we can get the weld joint by name.
  // As documented, a WeldJoint added with WeldFrames() will be named as:
  const std::string weld_name =
      plant.world_frame().name() + "_welds_to_" + base_link_frame.name();
  EXPECT_EQ(weld.name(), weld_name);
  EXPECT_NO_THROW(plant.GetJointByName(weld_name));
  EXPECT_EQ(plant.GetJointByName(weld_name).index(), weld.index());
  EXPECT_EQ(&plant.GetJointByName(weld_name), &weld);

  EXPECT_EQ(weld.num_positions(), 0);
  EXPECT_EQ(weld.num_velocities(), 0);

  // MultibodyPlant orders the state x with the vector q of generalized
  // positions followed by the vector v of generalized velocities.
  for (JointIndex joint_index(0);
       joint_index < plant.num_joints() - 1 /* Skip "weld" joint. */;
       ++joint_index) {
    const Joint<double>& joint = plant.get_joint(joint_index);
    // Start index in the vector q of generalized positions.
    const int expected_q_start = joint_index;
    // Start index in the vector v of generalized velocities.
    const int expected_v_start = joint_index;
    const int expected_num_v = 1;
    const int expected_num_q = 1;
    EXPECT_EQ(joint.num_positions(), expected_num_q);
    EXPECT_EQ(joint.position_start(), expected_q_start);
    EXPECT_EQ(joint.num_velocities(), expected_num_v);
    EXPECT_EQ(joint.velocity_start(), expected_v_start);

    // Confirm that the mutable accessor returns the same object.
    Joint<double>& mutable_joint = plant.get_mutable_joint(joint_index);
    EXPECT_EQ(&mutable_joint, &joint);
  }

  // Verify that the indexes above point to the right entries in the state
  // stored in the context.
  auto context = plant.CreateDefaultContext();

  for (JointIndex joint_index(1); /* Skip "weld_base_to_world". */
       joint_index < plant.num_joints(); ++joint_index) {
    // We know all joints in our model, besides the first joint welding the
    // model to the world, are revolute joints.
    const auto& joint = plant.GetJointByName<RevoluteJoint>(
        "iiwa_joint_" + std::to_string(joint_index));

    // We simply set each entry in the state with the value of its index.
    joint.set_angle(context.get(), joint.position_start());
    joint.set_angular_rate(context.get(),
                           plant.num_positions() + joint.velocity_start());
  }

  // Verify that each entry has the value we expect it to have.
  const VectorX<double> xc =
      context->get_continuous_state_vector().CopyToVector();
  const VectorX<double> xc_expected = VectorX<double>::LinSpaced(
      plant.num_multibody_states() /* size */,
      0 /* first index */, plant.num_multibody_states() - 1 /* last index */);

  EXPECT_EQ(xc, xc_expected);
}

// Unit test fixture for a model of Kuka Iiwa arm parametrized on the periodic
// update period of the plant. This allows us to test some of the plant's
// functionality for both continuous and discrete models.
class KukaArmTest : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() override {
    const char kSdfPath[] =
        "drake/manipulation/models/iiwa_description/sdf/"
            "iiwa14_no_collision.sdf";
    plant_ = std::make_unique<MultibodyPlant<double>>(this->GetParam());
    Parser(plant_.get()).AddModelFromFile(FindResourceOrThrow(kSdfPath));
    const Joint<double>& weld =
        plant_->WeldFrames(plant_->world_frame(),
                           plant_->GetFrameByName("iiwa_link_0"));
    plant_->Finalize();

    EXPECT_EQ(plant_->num_positions(), 7);
    EXPECT_EQ(plant_->num_velocities(), 7);

    // We expect the last joint to be the one WeldJoint fixing the model to the
    // world, since we added it last above with the call to WeldFrames().
    // We verify this assumption.
    ASSERT_EQ(weld.index(), plant_->num_joints() - 1);

    context_ = plant_->CreateDefaultContext();
  }

  // Helper to set the multibody state x to x[i] = i for each i-th entry in the
  // state vector.
  // We use RevoluteJoint's methods to set the state in order to independently
  // unit test the proper workings of
  // MultibodyTree::get_multibody_state_vector() and its mutable counterpart.
  void SetState(const VectorX<double>& xc) {
    const int nq = plant_->num_positions();
    for (JointIndex joint_index(0);
         joint_index < plant_->num_joints() - 1 /* Skip "weld" joint. */;
         ++joint_index) {
      // We know all joints in our model, besides the first joint welding the
      // model to the world, are revolute joints.
      const auto& joint = plant_->GetJointByName<RevoluteJoint>(
          "iiwa_joint_" + std::to_string(joint_index + 1));

      // For this simple model we do know the order in which variables are
      // stored in the state vector.
      const double angle = xc[joint_index];
      const double angle_rate = xc[nq + joint_index];

      // We simply set each entry in the state with the value of its index.
      joint.set_angle(context_.get(), angle);
      joint.set_angular_rate(context_.get(), angle_rate);
    }
  }

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
};

// This test verifies we can easily access the multibody state vector x = [q, v]
// for either a discrete or continuous multibody model.
TEST_P(KukaArmTest, StateAccess) {
  // Set the state to x[i] = i for each i-th entry.
  VectorX<double> xc_expected = VectorX<double>::LinSpaced(
      plant_->num_multibody_states() /* size */,
      1 /* first index */, plant_->num_multibody_states() /* last index */);
  SetState(xc_expected);

  // Verify that we can retrieve the state vector and that it has the values we
  // set above.
  // Note: xc is an Eigen block, that is, a reference to the values stored in
  // the context. Changes to state through the context can change the values
  // referenced by xc.
  Eigen::VectorBlock<const VectorX<double>> xc =
      plant_->GetPositionsAndVelocities(*context_);
  EXPECT_EQ(xc, xc_expected);

  // Modify positions and change xc expected to reflect changes to positions.
  for (int i = 0; i < plant_->num_positions(); ++i)
    xc_expected[i] *= -1;
  plant_->GetMutablePositions(context_.get()) =
      xc_expected.head(plant_->num_positions());
  EXPECT_EQ(plant_->GetPositions(*context_),
            xc_expected.head(plant_->num_positions()));
  EXPECT_EQ(xc, xc_expected);

  // SetPositions() should yield the same result.
  plant_->GetMutablePositions(context_.get()).setZero();
  plant_->SetPositions(
      context_.get(), xc_expected.head(plant_->num_positions()));
  EXPECT_EQ(plant_->GetPositions(*context_),
            xc_expected.head(plant_->num_positions()));

  // Modify velocities and change xc_expected to reflect changes to velocities.
  for (int i = 0; i < plant_->num_velocities(); ++i)
    xc_expected[i + plant_->num_positions()] *= -1;
  plant_->GetMutableVelocities(context_.get()) =
      xc_expected.tail(plant_->num_velocities());
  EXPECT_EQ(plant_->GetVelocities(*context_),
            xc_expected.tail(plant_->num_velocities()));
  EXPECT_EQ(xc, xc_expected);

  // SetVelocities() should yield the same result.
  plant_->GetMutableVelocities(context_.get()).setZero();
  plant_->SetVelocities(
      context_.get(), xc_expected.tail(plant_->num_velocities()));
  EXPECT_EQ(plant_->GetVelocities(*context_),
            xc_expected.tail(plant_->num_velocities()));
  EXPECT_EQ(xc, xc_expected);

  // Get a mutable state and modify it.
  // Note: xc above is referencing values stored in the context. Therefore
  // setting the entire state to zero changes the values referenced by xc.
  plant_->GetMutablePositionsAndVelocities(context_.get()).setZero();
  EXPECT_EQ(xc, VectorX<double>::Zero(plant_->num_multibody_states()));
  plant_->SetPositionsAndVelocities(context_.get(), xc_expected);
  EXPECT_EQ(xc, xc_expected);
}

TEST_P(KukaArmTest, InstanceStateAccess) {
  // Redo the setup process, now with two Iiwa's.
  const char kSdfPath[] =
      "drake/manipulation/models/iiwa_description/sdf/"
          "iiwa14_no_collision.sdf";
  plant_ = std::make_unique<MultibodyPlant<double>>(this->GetParam());
  Parser parser(plant_.get());
  multibody::ModelInstanceIndex arm1 = parser.AddModelFromFile(
      FindResourceOrThrow(kSdfPath), "arm1");
  multibody::ModelInstanceIndex arm2 = parser.AddModelFromFile(
      FindResourceOrThrow(kSdfPath), "arm2");
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("iiwa_link_0", arm1));
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("iiwa_link_0", arm2));
  plant_->Finalize();

  EXPECT_EQ(plant_->num_positions(), 14);
  EXPECT_EQ(plant_->num_velocities(), 14);

  // Re-create the context.
  context_ = plant_->CreateDefaultContext();

  // Prepare to set the positions, velocity, and state of one model instance.
  VectorX<double> q = VectorX<double>::LinSpaced(
      plant_->num_positions(arm2) /* size */,
      1 /* first number */, plant_->num_positions(arm2) /* last number */);
  VectorX<double> qd = VectorX<double>::LinSpaced(
      plant_->num_velocities(arm2) /* size */,
      10 /* first number */,
      9 + plant_->num_velocities(arm2) /* last number */);
  VectorX<double> x(q.size() + qd.size());
  x << q, qd;

  // Set the positions, make sure that they're retrieved successfully, and
  // verify that no other multibody instance positions or velocities are
  // altered.
  plant_->GetMutablePositionsAndVelocities(context_.get()).setZero();
  plant_->SetPositions(context_.get(), arm2, q);
  EXPECT_EQ(plant_->GetPositions(*context_, arm2), q);
  EXPECT_EQ(plant_->GetPositions(*context_, arm1).norm(), 0);
  EXPECT_EQ(plant_->GetVelocities(*context_, arm1).norm(), 0);
  EXPECT_EQ(plant_->GetVelocities(*context_, arm2).norm(), 0);

  // Set the velocities, make sure that they're retrieved successfully, and
  // verify that no other multibody instance positions or velocities are
  // altered.
  plant_->GetMutablePositionsAndVelocities(context_.get()).setZero();
  plant_->SetVelocities(context_.get(), arm2, qd);
  EXPECT_EQ(plant_->GetVelocities(*context_, arm2), qd);
  EXPECT_EQ(plant_->GetPositions(*context_, arm1).norm(), 0);
  EXPECT_EQ(plant_->GetVelocities(*context_, arm1).norm(), 0);
  EXPECT_EQ(plant_->GetPositions(*context_, arm2).norm(), 0);

  // Set the positions and velocities, make sure that they're retrieved
  // successfully and verify that no other multibody instance positions or
  // velocities are altered.
  plant_->GetMutablePositionsAndVelocities(context_.get()).setZero();
  plant_->SetPositionsAndVelocities(context_.get(), arm2, x);
  EXPECT_EQ(plant_->GetPositionsAndVelocities(*context_, arm2), x);
  EXPECT_EQ(plant_->GetPositionsAndVelocities(*context_, arm1).norm(), 0);
}

// Verifies we instantiated an appropriate MultibodyPlant model based on the
// fixture's parameter.
TEST_P(KukaArmTest, CheckContinuousOrDiscreteModel) {
  // The plant must be a discrete system if the periodic update period is zero.
  EXPECT_EQ(!plant_->is_discrete(), this->GetParam() == 0);
}

INSTANTIATE_TEST_CASE_P(
    Blank, KukaArmTest,
    testing::Values(0.0 /* continuous state */, 1e-3 /* discrete state */));

GTEST_TEST(StateSelection, JointHasNoActuator) {
  const std::string file_name =
      "drake/multibody/benchmarks/acrobot/acrobot.sdf";
  MultibodyPlant<double> plant;
  Parser(&plant).AddModelFromFile(FindResourceOrThrow(file_name));
  plant.Finalize();

  // Sanity checks.
  ASSERT_EQ(plant.num_positions(), 2);
  ASSERT_EQ(plant.num_velocities(), 2);
  ASSERT_EQ(plant.num_actuators(), 1);  // ShoulderJoint is not actuated.

  // Attempt to make an actuation selector matrix for ShoulderJoint, which is
  // not actuated.
  std::vector<JointIndex> selected_joints;
  selected_joints.push_back(plant.GetJointByName("ShoulderJoint").index());
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.MakeActuatorSelectorMatrix(selected_joints),
      std::logic_error,
      "Joint 'ShoulderJoint' does not have an actuator.");
}

GTEST_TEST(StateSelection, KukaWithSimpleGripper) {
  const char kArmSdfPath[] =
      "drake/manipulation/models/iiwa_description/sdf/"
          "iiwa14_no_collision.sdf";

  const char kWsg50SdfPath[] =
      "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf";

  // We make a "floating" model of a Kuka arm with a Schunk WSG 50 gripper at
  // the end effector. The purpose of having this floating model is to unit test
  // the methods for making state/actuation selector matrices for more complex
  // cases when we have floating robots in the model.
  MultibodyPlant<double> plant;
  Parser parser(&plant);
  const ModelInstanceIndex arm_model =
      parser.AddModelFromFile(FindResourceOrThrow(kArmSdfPath));

  // Add the gripper.
  const ModelInstanceIndex gripper_model =
      parser.AddModelFromFile(FindResourceOrThrow(kWsg50SdfPath));
  const auto& end_effector = plant.GetBodyByName("iiwa_link_7", arm_model);
  const auto& gripper_body = plant.GetBodyByName("body", gripper_model);
  // We don't care for the actual pose of the gripper in the end effector frame
  // for this example. We only care about the state size.
  plant.WeldFrames(end_effector.body_frame(), gripper_body.body_frame());
  plant.Finalize();

  // Sanity check basic invariants.
  const int num_floating_positions = 7;
  const int num_floating_velocities = 6;

  // Seven dofs for the arm and two for the gripper, plus floating base.
  EXPECT_EQ(plant.num_positions(), 9 + num_floating_positions);
  EXPECT_EQ(plant.num_velocities(), 9 + num_floating_velocities);

  // Selected joints by name.
  const std::vector<std::string> arm_selected_joints_by_name =
      {"iiwa_joint_2", "iiwa_joint_7", "iiwa_joint_3"};

  std::vector<JointIndex> arm_selected_joints;
  // For this example we are only interested in the state for joints:
  //  - iiwa_joint_2
  //  - iiwa_joint_7
  //  - iiwa_joint_3
  // In that order.
  // We therefore create a user to joint index map accordingly.
  for (const auto& joint_name : arm_selected_joints_by_name) {
    arm_selected_joints.push_back(
        plant.GetJointByName(joint_name).index());
  }

  // State selector for the arm.
  const MatrixX<double> Sx_arm =
      plant.MakeStateSelectorMatrix(arm_selected_joints);

  // Actuation selector for the arm.
  const MatrixX<double> Su_arm =
      plant.MakeActuatorSelectorMatrix(arm_selected_joints);

  // Verify the sizes (all these joints are revolute with one q and one v).
  const int num_selected_states = 2 * arm_selected_joints.size();
  const int num_states = plant.num_multibody_states();
  ASSERT_EQ(Sx_arm.rows(), num_selected_states);
  ASSERT_EQ(Sx_arm.cols(), num_states);

  // We know what the selection matrix should be for this case:
  const int nq = plant.num_positions();
  MatrixX<double> Sx_arm_expected =
      MatrixX<double>::Zero(num_selected_states, plant.num_multibody_states());
  // position for first joint, iiwa_joint_2.
  Sx_arm_expected(0, num_floating_positions + 1) = 1;
  // position for second joint, iiwa_joint_7.
  Sx_arm_expected(1, num_floating_positions + 6) = 1;
  // position for third joint, iiwa_joint_3.
  Sx_arm_expected(2, num_floating_positions + 2) = 1;
  // velocity for first joint, iiwa_joint_2.
  Sx_arm_expected(3, num_floating_velocities + nq + 1) = 1;
  // velocity for second joint, iiwa_joint_7.
  Sx_arm_expected(4, num_floating_velocities + nq + 6) = 1;
  // velocity for third joint, iiwa_joint_3.
  Sx_arm_expected(5, num_floating_velocities + nq + 2) = 1;
  EXPECT_EQ(Sx_arm, Sx_arm_expected);

  auto OldMakeStateSelectorMatrixFromJointNames = [&plant](auto names) {
    // TODO(eric.cousineau): Move this to `multibody_tree_test`, or remove it
    // once it becomes internal.
    const auto& tree = internal::GetInternalTree(plant);
    return tree.MakeStateSelectorMatrixFromJointNames(names);
  };

  // State selection using alternative API in which joints are specified by
  // name.
  const MatrixX<double> Sx_arm_by_name =
      OldMakeStateSelectorMatrixFromJointNames(
          arm_selected_joints_by_name);
  EXPECT_EQ(Sx_arm_by_name, Sx_arm_expected);

  // Intentionally attempt to create a state selector from a vector with
  // repeated joint names in order to verify the method throws.
  const std::vector<std::string> repeated_joint_names =
      {"iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_7", "iiwa_joint_3"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      OldMakeStateSelectorMatrixFromJointNames(repeated_joint_names),
      std::logic_error,
      "Joint named 'iiwa_joint_3' is repeated multiple times.");

  // Intentionally attempt to create a state selector from a vector with
  // repeated joint indexes in order to verify the method throws.
  std::vector<JointIndex> repeated_joint_indexes;
  for (const auto& joint_name : repeated_joint_names) {
    repeated_joint_indexes.push_back(plant.GetJointByName(joint_name).index());
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.MakeStateSelectorMatrix(repeated_joint_indexes),
      std::logic_error,
      "Joint named 'iiwa_joint_3' is repeated multiple times.");

  // Verify the arm's actuation selector.
  MatrixX<double> Su_arm_expected =
      MatrixX<double>::Zero(plant.num_actuators(), 3);
  Su_arm_expected(1, 0) = 1;  // Actuator on first joint, iiwa_joint_2.
  Su_arm_expected(6, 1) = 1;  // Actuator on second joint, iiwa_joint_7.
  Su_arm_expected(2, 2) = 1;  // Actuator on third joint, iiwa_joint_3.
  EXPECT_EQ(Su_arm, Su_arm_expected);

  // We build a state selector for the gripper dofs.
  std::vector<JointIndex> gripper_selected_joints;
  gripper_selected_joints.push_back(plant.GetJointByName(
      "left_finger_sliding_joint").index());  // user index = 0.
  gripper_selected_joints.push_back(plant.GetJointByName(
      "right_finger_sliding_joint").index());  // user index = 1.

  // State selector for the griper.
  const MatrixX<double> Sx_gripper =
      plant.MakeStateSelectorMatrix(gripper_selected_joints);

  // Actuation selector for the gripper.
  const MatrixX<double> Su_gripper =
      plant.MakeActuatorSelectorMatrix(gripper_selected_joints);

  // Similarly we know the expected value for the gripper's selector matrix.
  MatrixX<double> Sx_gripper_expected =
      MatrixX<double>::Zero(4, plant.num_multibody_states());
  // first joint position, left finger.
  Sx_gripper_expected(0, num_floating_positions + 7) = 1;
  // second joint position, right finger.
  Sx_gripper_expected(1, num_floating_positions + 8) = 1;
  // first joint velocity, left finger.
  Sx_gripper_expected(2, num_floating_velocities + nq + 7) = 1;
  // second joint velocity, right finger.
  Sx_gripper_expected(3, num_floating_velocities + nq + 8) = 1;
  EXPECT_EQ(Sx_gripper, Sx_gripper_expected);

  // Verify the grippers's actuation selector.
  MatrixX<double> Su_gripper_expected =
      MatrixX<double>::Zero(plant.num_actuators(), 2);
  Su_gripper_expected(7, 0) = 1;  // Actuator on first joint, left finger.
  Su_gripper_expected(8, 1) = 1;  // Actuator on second joint, right finger.
  EXPECT_EQ(Su_gripper, Su_gripper_expected);

  // Verify we can make selector matrices from empty lists of joints/actuators.
  const MatrixX<double> Sx_from_empty_names =
      OldMakeStateSelectorMatrixFromJointNames(
          std::vector<std::string>());
  EXPECT_EQ(Sx_from_empty_names.rows(), 0);
  EXPECT_EQ(Sx_from_empty_names.cols(), plant.num_multibody_states());

  const MatrixX<double> Sx_from_empty_indexes =
      plant.MakeStateSelectorMatrix(std::vector<JointIndex>());
  EXPECT_EQ(Sx_from_empty_indexes.rows(), 0);
  EXPECT_EQ(Sx_from_empty_indexes.cols(), plant.num_multibody_states());

  const MatrixX<double> Su_from_empty_actuators =
      plant.MakeActuatorSelectorMatrix(
          std::vector<JointActuatorIndex>());
  EXPECT_EQ(Su_from_empty_actuators.rows(), plant.num_actuators());
  EXPECT_EQ(Su_from_empty_actuators.cols(), 0);

  const MatrixX<double> Su_from_empty_joints =
      plant.MakeActuatorSelectorMatrix(std::vector<JointIndex>());
  EXPECT_EQ(Su_from_empty_actuators.rows(), plant.num_actuators());
  EXPECT_EQ(Su_from_empty_actuators.cols(), 0);

  // Verify we can make an actuation matrix for this model.
  const int nv = plant.num_velocities();
  const int nu = plant.num_actuators();
  ASSERT_EQ(nv, 15);  // 6 free dofs + 7 kuka joints + 2 wsg fingers.
  ASSERT_EQ(nu, 9);   // 7 kuka joints + 2 wsg fingers.
  const MatrixX<double> B = plant.MakeActuationMatrix();
  ASSERT_EQ(B.rows(), nv);
  ASSERT_EQ(B.cols(), nu);
  MatrixX<double> B_expected = MatrixX<double>::Zero(nv, nu);
  auto B_iiwa = B_expected.block(
      6 /* skip floating base */, 0,
      7 /* iiwa joints */, 7 /* iiwa actuators */);
  auto B_wsg = B_expected.block(
      13 /* skip iiwa dofs */, 7 /* skip iiwa actuators */,
      2 /* wsg joints */, 2 /* wsg actuators */);
  B_iiwa.setIdentity();
  B_wsg.setIdentity();
  EXPECT_TRUE(CompareMatrices(B, B_expected, 0.0, MatrixCompareType::absolute));

  // Test old spellings.
  unused(plant.MakeStateSelectorMatrix(std::vector<JointIndex>()));
  unused(plant.MakeActuatorSelectorMatrix(std::vector<JointIndex>()));
  unused(plant.MakeActuatorSelectorMatrix(
      std::vector<JointActuatorIndex>()));
}

// This unit test verifies the workings of
// MBP::SetFreeBodyPoseInAnchoredFrame(). To that end we build a model
// representative of a real setup consisting of a robot arm mounted on a robot
// table, an objects table and a mug. This test defines an objects frame O with
// its origin located a the -x, -y corner of the objects table. With this setup,
// we test we can set the pose X_OM of the mug frame M in the objects frame O.
GTEST_TEST(StateSelection, FloatingBodies) {
  const std::string iiwa_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/"
          "iiwa14_no_collision.sdf");

  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/"
          "extra_heavy_duty_table_surface_only_collision.sdf");

  const std::string mug_sdf_path = FindResourceOrThrow(
      "drake/examples/simple_gripper/simple_mug.sdf");

  MultibodyPlant<double> plant;

  // Load a model of a table for the robot.
  Parser parser(&plant);
  const ModelInstanceIndex robot_table_model =
      parser.AddModelFromFile(table_sdf_path, "robot_table");
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", robot_table_model));

  // Load the robot and weld it on top of the robot table.
  const ModelInstanceIndex arm_model =
      parser.AddModelFromFile(iiwa_sdf_path);

  const double table_top_z_in_world =
      // table's top height
      0.736 +
      // table's top width
      0.057 / 2;
  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("iiwa_link_0", arm_model),
      RigidTransform<double>(Vector3d(0, 0, table_top_z_in_world))
          .GetAsIsometry3());

  // Load a second table for objects.
  const ModelInstanceIndex objects_table_model =
      parser.AddModelFromFile(table_sdf_path, "objects_table");
  const Isometry3d X_WT(Translation3d(0.8, 0.0, 0.0));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", objects_table_model), X_WT);

  // Define a fixed frame on the -x, -y corner of the objects table.
  const Isometry3d X_TO = RigidTransform<double>(
      RotationMatrix<double>::MakeXRotation(-M_PI_2),
      Vector3<double>(-0.3, -0.3, table_top_z_in_world)).GetAsIsometry3();
  const auto& objects_frame_O =
      plant.AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          "objects_frame", plant.GetFrameByName("link", objects_table_model),
          X_TO));

  // Add a floating mug.
  const ModelInstanceIndex mug_model =
      parser.AddModelFromFile(mug_sdf_path);
  const Body<double>& mug = plant.GetBodyByName("main_body", mug_model);

  plant.Finalize();

  // Check link 0 is anchored, and link 1 is not.
  EXPECT_TRUE(plant.IsAnchored(plant.GetBodyByName("iiwa_link_0", arm_model)));
  EXPECT_FALSE(
      plant.IsAnchored(plant.GetBodyByName("iiwa_link_1", arm_model)));

  auto context = plant.CreateDefaultContext();

  // Initialize the pose X_OM of the mug frame M in the objects table frame O.
  const Isometry3d X_OM(Translation3d(0.05, 0.0, 0.05));
  plant.SetFreeBodyPoseInAnchoredFrame(
      context.get(), objects_frame_O, mug, X_OM);

  // Retrieve the pose of the mug in the world.
  const Isometry3d& X_WM = plant.EvalBodyPoseInWorld(*context, mug);

  const Isometry3d X_WM_expected = X_WT * X_TO * X_OM;

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_WM.matrix(), X_WM_expected.matrix(),
                              kTolerance, MatrixCompareType::relative));

  // SetFreeBodyPoseInAnchoredFrame() should throw if the reference frame F is
  // not anchored to the world.
  const Frame<double>& end_effector_frame =
      plant.GetFrameByName("iiwa_link_7", arm_model);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.SetFreeBodyPoseInAnchoredFrame(
          context.get(), end_effector_frame, mug, X_OM),
      std::logic_error,
      "Frame 'iiwa_link_7' must be anchored to the world frame.");
}

GTEST_TEST(SetRandomTest, FloatingBodies) {
  // Create a model that contains a single body.
  MultibodyPlant<double> plant;
  const Body<double>& body =
      plant.AddRigidBody("LoneBody", SpatialInertia<double>());
  plant.Finalize();

  RandomGenerator generator;

  std::uniform_real_distribution<symbolic::Expression> uniform;
  Vector3<symbolic::Expression> xyz(1.0 + uniform(generator),
                                    2.0 + uniform(generator),
                                    3.0 + uniform(generator));

  plant.SetFreeBodyRandomPositionDistribution(body, xyz);
  plant.SetFreeBodyRandomRotationDistributionToUniform(body);

  auto context = plant.CreateDefaultContext();

  const math::RigidTransform<double> X_WB_default =
      plant.GetFreeBodyPose(*context, body);

  plant.SetRandomContext(context.get(), &generator);
  math::RigidTransform<double> X_WB =
      plant.GetFreeBodyPose(*context, body);

  // Just make sure that the rotation matrices have changed. (Testing that
  // the quaternion is unit norm would be good, but is not possible here
  // because the RigidTransform class scales the quaternion before it is
  // returned).
  EXPECT_FALSE(CompareMatrices(X_WB_default.rotation().matrix(),
                               X_WB.rotation().matrix()));

  // x is drawn from [1, 2).
  EXPECT_GE(X_WB.translation()[0], 1.0);
  EXPECT_LT(X_WB.translation()[0], 2.0);

  // y is drawn from [2, 3).
  EXPECT_GE(X_WB.translation()[1], 2.0);
  EXPECT_LT(X_WB.translation()[1], 3.0);

  // z is drawn from [3, 4).
  EXPECT_GE(X_WB.translation()[2], 3.0);
  EXPECT_LT(X_WB.translation()[2], 4.0);

  // Check that we can set the rotation to a specific distribution (in this
  // case it's just constant).
  const math::RotationMatrix<double> X_WB_new(
      math::RollPitchYaw<double>(0.3, 0.4, 0.5));
  plant.SetFreeBodyRandomRotationDistribution(
      body, X_WB_new.cast<symbolic::Expression>().ToQuaternion());

  plant.SetRandomContext(context.get(), &generator);
  X_WB = plant.GetFreeBodyPose(*context, body);

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(
      X_WB_new.matrix(), X_WB.rotation().matrix(),
      kTolerance, MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
