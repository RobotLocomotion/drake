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
#include "drake/common/nice_type_name.h"
#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/test_utilities/dummy_render_engine.h"
#include "drake/geometry/test_utilities/geometry_set_tester.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/discrete_contact_pair.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/test_utilities/add_fixed_objects_to_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {

using Eigen::AngleAxisd;
using Eigen::Matrix2d;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryId;
using geometry::IllustrationProperties;
using geometry::internal::DummyRenderEngine;
using geometry::PenetrationAsPointPair;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrix;
using math::RotationMatrixd;
using multibody::benchmarks::Acrobot;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::pendulum::MakePendulumPlant;
using multibody::benchmarks::pendulum::PendulumParameters;
using multibody::MultibodyForces;
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
    // We first convert point contact pairs to discrete contact pairs.
    std::vector<internal::DiscreteContactPair<double>> discrete_pairs;
    for (const PenetrationAsPointPair<double>& pair : point_pairs) {
      const Vector3d p_WC = 0.5 * (pair.p_WCa + pair.p_WCb);
      // fn0, k and d are irrelevant values for Jacobian computation. Thus we
      // arbitrarily set them to zero.
      const double fn0 = 0.0;
      const double k = 0.0;
      const double d = 0.0;
      discrete_pairs.push_back(
          {pair.id_A, pair.id_B, p_WC, pair.nhat_BA_W, fn0, k, d});
    }
    plant.CalcNormalAndTangentContactJacobians(
        context, discrete_pairs, Jn, Jt, R_WC_set);
  }

  static const geometry::QueryObject<double>& EvalGeometryQueryInput(
      const MultibodyPlant<double>& plant,
      const systems::Context<double>& context) {
    return plant.EvalGeometryQueryInput(context);
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

  // Verify that a non-positive penetration_allowance throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->set_penetration_allowance(-1), std::logic_error,
      "set_penetration_allowance\\(\\): penetration_allowance must be strictly "
      "positive.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->set_penetration_allowance(0), std::logic_error,
      "set_penetration_allowance\\(\\): penetration_allowance must be strictly "
      "positive.");

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
  EXPECT_EQ(plant->get_state_output_port().size(), 6);
  EXPECT_EQ(plant->get_state_output_port(
      default_model_instance()).size(), 4);
  EXPECT_EQ(plant->get_state_output_port(
      pendulum_model_instance).size(), 2);

  // Check that model-instance ports get named properly.
  EXPECT_TRUE(plant->HasModelInstanceNamed("DefaultModelInstance"));
  EXPECT_TRUE(plant->HasModelInstanceNamed("SplitPendulum"));
  EXPECT_EQ(
      plant->get_actuation_input_port(default_model_instance()).get_name(),
      "DefaultModelInstance_actuation");
  EXPECT_EQ(plant->get_state_output_port(default_model_instance())
                .get_name(),
            "DefaultModelInstance_continuous_state");
  EXPECT_EQ(plant->get_state_output_port(pendulum_model_instance)
                .get_name(),
            "SplitPendulum_continuous_state");

  // Query if elements exist in the model.
  EXPECT_TRUE(plant->HasBodyNamed(parameters.link1_name()));
  EXPECT_TRUE(plant->HasBodyNamed(parameters.link2_name()));
  EXPECT_FALSE(plant->HasBodyNamed(kInvalidName));
  // Indicator that the plant is calling the tree's method correctly.
  EXPECT_EQ(plant->NumBodiesWithName(parameters.link1_name()), 1);
  EXPECT_EQ(plant->NumBodiesWithName(kInvalidName), 0);

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
  const Joint<double>& weld_joint =
      plant->GetJointByName("weld");
  EXPECT_EQ(weld_joint.model_instance(), pendulum_model_instance);
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // Get force elements by index. In this case the default gravity field.
  const ForceElementIndex gravity_field_index(0);
  EXPECT_EQ(
      &plant->gravity_field(),
      &plant->GetForceElement<UniformGravityFieldElement>(gravity_field_index));
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->GetForceElement<RevoluteSpring>(gravity_field_index),
      std::logic_error,
      ".*not of type .*RevoluteSpring.* but of type "
      ".*UniformGravityFieldElement.*");
  const ForceElementIndex invalid_force_index(plant->num_force_elements() + 1);
  EXPECT_ANY_THROW(plant->GetForceElement<RevoluteSpring>(invalid_force_index));

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
  EXPECT_EQ(pendulum_joint_indices[1], weld_joint.index());

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

  // Templatized version throws when the requested joint doesn't have the
  // expected type.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->GetJointByName<PrismaticJoint>(parameters.shoulder_joint_name(),
                                            shoulder.model_instance()),
      std::logic_error,
      ".*not of type .*PrismaticJoint.* but of type "
      ".*RevoluteJoint.*");

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
          "AnotherJoint", link1, std::nullopt, link2, std::nullopt,
          Vector3d::UnitZ()),
      std::logic_error,
      "Post-finalize calls to '.*' are not allowed; "
      "calls to this method must happen before Finalize\\(\\).");
  // Test API for simplified `AddJoint` method.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->AddJoint(std::make_unique<RevoluteJoint<double>>(
          "AnotherJoint", link1.body_frame(), link2.body_frame(),
          Vector3d::UnitZ())),
      std::logic_error,
      "Post-finalize calls to '.*' are not allowed; "
      "calls to this method must happen before Finalize\\(\\).");
  // TODO(amcastro-tri): add test to verify that requesting a joint of the wrong
  // type throws an exception. We need another joint type to do so.

  // Get frame indices by model_instance
  const std::vector<FrameIndex> acrobot_frame_indices =
      plant->GetFrameIndices(default_model_instance());
  ASSERT_EQ(acrobot_frame_indices.size(), 3);
  EXPECT_EQ(acrobot_frame_indices[0], link1.body_frame().index());
  EXPECT_EQ(acrobot_frame_indices[1], link2.body_frame().index());
  EXPECT_EQ(acrobot_frame_indices[2], elbow_joint.frame_on_parent().index());

  const Frame<double>& model_frame = plant->GetFrameByName("__model__");
  EXPECT_EQ(model_frame.model_instance(), pendulum_model_instance);
  const std::vector<FrameIndex> pendulum_frame_indices =
      plant->GetFrameIndices(pendulum_model_instance);
  ASSERT_EQ(pendulum_frame_indices.size(), 6);
  EXPECT_EQ(pendulum_frame_indices[0], upper.body_frame().index());
  EXPECT_EQ(pendulum_frame_indices[1], lower.body_frame().index());
  EXPECT_EQ(pendulum_frame_indices[2], model_frame.index());
  EXPECT_EQ(pendulum_frame_indices[3], pin_joint.frame_on_child().index());
  EXPECT_EQ(pendulum_frame_indices[4], weld_joint.frame_on_parent().index());
  EXPECT_EQ(pendulum_frame_indices[5], weld_joint.frame_on_child().index());
}

GTEST_TEST(MultibodyPlantTest, AddMultibodyPlantSceneGraph) {
  systems::DiagramBuilder<double> builder;
  auto pair = AddMultibodyPlantSceneGraph(&builder, 0.0);

  MultibodyPlant<double>* plant{};
  geometry::SceneGraph<double>* scene_graph{};
  // Check `tie` assignment.
  std::tie(plant, scene_graph) = pair;
  EXPECT_NE(plant, nullptr);
  EXPECT_NE(scene_graph, nullptr);
  // Check referencing.
  MultibodyPlant<double>& plant_ref = pair;
  EXPECT_EQ(&plant_ref, plant);

  // Check support for C++17's structured binding.
  auto [first_element, second_element] =
      AddMultibodyPlantSceneGraph(&builder, 0.0);
  // Verify the expected types.
  EXPECT_EQ(drake::NiceTypeName::Get(first_element),
            drake::NiceTypeName::Get<MultibodyPlant<double>>());
  EXPECT_EQ(drake::NiceTypeName::Get(second_element),
            drake::NiceTypeName::Get<SceneGraph<double>>());

  // These should fail:
  // AddMultibodyPlantSceneGraphResult<double> extra(plant, scene_graph);
  // AddMultibodyPlantSceneGraphResult<double> extra{*plant, *scene_graph};
}

// Verifies that string-based queries allocate no heap.
//
// NOTE: Every querying API that takes strings as parameters should conform to
// this non-allocating convention. As such, they should be invoked in this
// test to show that they adhere to that expectation. It should be considered
// to be a defect if such an API exists and is omitted here (for whatever
// historical reason).
GTEST_TEST(MultibodyPlantTest, NoHeapAllocOnStringQueries) {
  // Construct a plant with an Iiwa.
  const char kSdfPath[] =
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf";
  auto plant =
      std::make_unique<MultibodyPlant<double>>(0 /* plant type irrelevant */);
  Parser parser(plant.get());
  multibody::ModelInstanceIndex iiwa_instance =
      parser.AddModelFromFile(FindResourceOrThrow(kSdfPath), "iiwa");
  plant->Finalize();

  // Use string to ensure that there is no heap allocation in the implicit
  // conversion to string_view.
  const std::string kLinkName = "iiwa_link_0";
  const std::string kJointName = "iiwa_joint_1";

  // Note that failed queries cause exceptions to be thrown (which allocates
  // heap).
  drake::test::LimitMalloc dummy;

  // Check the HasX versions first. Note that functions that take no model
  // instance argument delgate to model instance argument versions.
  EXPECT_TRUE(plant->HasModelInstanceNamed("iiwa"));
  EXPECT_TRUE(plant->HasBodyNamed(kLinkName, iiwa_instance));
  EXPECT_TRUE(plant->HasFrameNamed(kLinkName, iiwa_instance));
  EXPECT_TRUE(plant->HasJointNamed(kJointName, iiwa_instance));
  EXPECT_TRUE(plant->HasJointActuatorNamed(kJointName, iiwa_instance));

  // Check the GetX versions now.
  plant->GetModelInstanceByName("iiwa");
  plant->GetBodyByName(kLinkName, iiwa_instance);
  plant->GetFrameByName(kLinkName, iiwa_instance);
  plant->GetJointByName(kJointName, iiwa_instance);
  plant->GetJointActuatorByName(kJointName, iiwa_instance);
}

GTEST_TEST(MultibodyPlantTest, EmptyWorldDiscrete) {
  const double discrete_update_period = 1.0e-3;
  MultibodyPlant<double> plant(discrete_update_period);
  plant.Finalize();
  EXPECT_EQ(plant.num_velocities(), 0);
  EXPECT_EQ(plant.num_positions(), 0);
  // Compute discrete update.
  auto context = plant.CreateDefaultContext();
  auto& discrete_state_vector = context->get_discrete_state_vector();
  EXPECT_EQ(discrete_state_vector.size(), 0);
  auto new_discrete_state = plant.AllocateDiscreteVariables();
  const systems::VectorBase<double>& new_discrete_state_vector =
      new_discrete_state->get_vector();
  EXPECT_EQ(new_discrete_state_vector.size(), 0);
  DRAKE_EXPECT_NO_THROW(
      plant.CalcDiscreteVariableUpdates(*context, new_discrete_state.get()));
}

GTEST_TEST(MultibodyPlantTest, EmptyWorldContinuous) {
  MultibodyPlant<double> plant(0.0);
  plant.Finalize();
  EXPECT_EQ(plant.num_velocities(), 0);
  EXPECT_EQ(plant.num_positions(), 0);
  // Compute continuous derivatives.
  auto context = plant.CreateDefaultContext();
  auto& continuous_state_vector = context->get_continuous_state_vector();
  EXPECT_EQ(continuous_state_vector.size(), 0);
  auto new_derivatives = plant.AllocateTimeDerivatives();
  EXPECT_EQ(new_derivatives->size(), 0);
  DRAKE_EXPECT_NO_THROW(
      plant.CalcTimeDerivatives(*context, new_derivatives.get()));
  VectorXd residual = plant.AllocateImplicitTimeDerivativesResidual();
  EXPECT_EQ(residual.size(), 0);
  DRAKE_EXPECT_NO_THROW(plant.CalcImplicitTimeDerivativesResidual(
      *context, *new_derivatives, &residual));
}

GTEST_TEST(ActuationPortsTest, CheckActuation) {
  // Create a MultibodyPlant consisting of two model instances, one actuated
  // and the other unactuated.
  MultibodyPlant<double> plant(0.0);
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

  // Verify which bodies are free and modeled with quaternions.
  EXPECT_FALSE(plant.GetBodyByName("Link1").is_floating());
  EXPECT_FALSE(plant.GetBodyByName("Link1").has_quaternion_dofs());
  EXPECT_FALSE(plant.GetBodyByName("Link2").is_floating());
  EXPECT_FALSE(plant.GetBodyByName("Link2").has_quaternion_dofs());
  EXPECT_TRUE(plant.GetBodyByName("uniformSolidCylinder").is_floating());
  EXPECT_TRUE(
      plant.GetBodyByName("uniformSolidCylinder").has_quaternion_dofs());

  // Verify that we can get the actuation input ports.
  DRAKE_EXPECT_NO_THROW(plant.get_actuation_input_port());
  DRAKE_EXPECT_NO_THROW(plant.get_actuation_input_port(acrobot_instance));
  DRAKE_EXPECT_NO_THROW(plant.get_actuation_input_port(cylinder_instance));

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
  plant.get_actuation_input_port(acrobot_instance).FixValue(context.get(), 0.0);
  DRAKE_EXPECT_NO_THROW(
      plant.CalcTimeDerivatives(*context, continuous_state.get()));

  // Verify that derivatives can be computed after fixing the cylinder actuation
  // input port with an empty vector.
  plant.get_actuation_input_port(cylinder_instance)
      .FixValue(context.get(), VectorXd(0));
  DRAKE_EXPECT_NO_THROW(
      plant.CalcTimeDerivatives(*context, continuous_state.get()));
}

GTEST_TEST(MultibodyPlant, UniformGravityFieldElementTest) {
  MultibodyPlant<double> plant(0.0);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddForceElement<UniformGravityFieldElement>(
          Vector3d(-1, 0, 0)),
      std::runtime_error,
      "This model already contains a gravity field element.*");
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
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, 0.0);
    Parser(plant_).AddModelFromFile(full_name);
    // Sanity check on the availability of the optional source id before using
    // it.
    DRAKE_DEMAND(plant_->get_source_id() != std::nullopt);

    // Ensure that we can access the geometry ports pre-finalize.
    DRAKE_EXPECT_NO_THROW(plant_->get_geometry_query_input_port());
    DRAKE_EXPECT_NO_THROW(plant_->get_geometry_poses_output_port());

    DRAKE_EXPECT_THROWS_MESSAGE(
        plant_->get_state_output_port(),
        std::logic_error,
        /* Verify this method is throwing for the right reasons. */
        "Pre-finalize calls to '.*' are not allowed; "
        "you must call Finalize\\(\\) first.");

    link1_ = &plant_->GetBodyByName(parameters_.link1_name());
    link2_ = &plant_->GetBodyByName(parameters_.link2_name());

    // Test we can call these methods pre-finalize.
    const FrameId link1_frame_id =
        plant_->GetBodyFrameIdOrThrow(link1_->index());
    EXPECT_EQ(link1_frame_id, *plant_->GetBodyFrameIdIfExists(link1_->index()));
    EXPECT_EQ(plant_->GetBodyFromFrameId(link1_frame_id), link1_);

    // Finalize() the plant.
    plant_->Finalize();

    // And build the Diagram:
    diagram_ = builder.Build();

    shoulder_ = &plant_->GetMutableJointByName<RevoluteJoint>(
        parameters_.shoulder_joint_name());
    elbow_ = &plant_->GetMutableJointByName<RevoluteJoint>(
        parameters_.elbow_joint_name());

    context_ = diagram_->CreateDefaultContext();
    derivatives_ = diagram_->AllocateTimeDerivatives();
    plant_context_ = &diagram_->GetMutableSubsystemContext(
        *plant_, context_.get());

    ASSERT_GT(plant_->num_actuators(), 0);
    input_port_ =
        &plant_->get_actuation_input_port().FixValue(plant_context_, 0.0);
  }

  void SetUpDiscreteAcrobotPlant(double time_step) {
    systems::DiagramBuilder<double> builder;
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/benchmarks/acrobot/acrobot.sdf");
    discrete_plant_ = std::make_unique<MultibodyPlant<double>>(time_step);
    Parser(discrete_plant_.get()).AddModelFromFile(full_name);
    discrete_plant_->Finalize();

    discrete_context_ = discrete_plant_->CreateDefaultContext();
    ASSERT_EQ(discrete_plant_->num_actuators(), 1);
    discrete_plant_->get_actuation_input_port().FixValue(
        discrete_context_.get(), 0.0);

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

    // Set arbitrary non-zero velocities to test they do not affect the results.
    shoulder_->set_angular_rate(plant_context_, 10.0);
    elbow_->set_angular_rate(plant_context_, 10.0);

    // Calculate the generalized forces due to gravity.
    const VectorX<double> tau_g =
        plant_->CalcGravityGeneralizedForces(*plant_context_);

    // Calculate a benchmark value.
    const Vector2d tau_g_expected =
        acrobot_benchmark_.CalcGravityVector(theta1, theta2);

    EXPECT_TRUE(CompareMatrices(
        tau_g, tau_g_expected, kTolerance, MatrixCompareType::relative));

    // Alternatively, we can use CalcForceElementsContribution().
    MultibodyForces<double> forces(*plant_);
    plant_->CalcForceElementsContribution(*plant_context_, &forces);
    // N.B. For this particular model we know `forces` only includes generalized
    // forces due to dissipation at the joints and spatial forces due to
    // gravity. In general after this call, MultibodyForces will contain a soup
    // of all forces stemming from a ForceElement making it impossible to
    // discern separate contributions. Therefore we should always use
    // ForceElement's API to retrieve the specific component we are interested
    // in. Here we access the internal data in MultibodyForces directly for
    // testing purposes only, a pattern not to be followed.

    // We verify first the body spatial forces include gravity. For this case,
    // it is all they should include. Skip the world, index = 0.
    const Vector3<double> gacc = plant_->gravity_field().gravity_vector();
    for (BodyIndex body_index(1); body_index < plant_->num_bodies();
         ++body_index) {
      const Body<double>& body = plant_->get_body(body_index);
      const SpatialForce<double>& F_Bo_W =
          body.GetForceInWorld(*plant_context_, forces);
      const double mass = body.get_default_mass();
      // TODO(amcastro-tri): provide Body::EvalCOMInWorld().
      const Vector3<double> p_BoBcm_B =
          body.CalcCenterOfMassInBodyFrame(*plant_context_);
      const RigidTransform<double> X_WB =
          plant_->EvalBodyPoseInWorld(*plant_context_, body);
      const RotationMatrix<double> R_WB = X_WB.rotation();
      const Vector3<double> p_BoBcm_W = R_WB * p_BoBcm_B;

      const Vector3<double> f_Bo_W_expected = mass * gacc;
      const Vector3<double> t_Bo_W_expected = p_BoBcm_W.cross(f_Bo_W_expected);
      EXPECT_TRUE(CompareMatrices(F_Bo_W.translational(), f_Bo_W_expected,
                                  kTolerance, MatrixCompareType::relative));
      EXPECT_TRUE(CompareMatrices(F_Bo_W.rotational(), t_Bo_W_expected,
                                  kTolerance, MatrixCompareType::relative));
    }

    // Now we'll use inverse dynamics to obtain the generalized forces
    // contribution due to gravity.

    // Zero generalized forces due to joint dissipation.
    forces.mutable_generalized_forces().setZero();

    // Zero accelerations.
    const VectorX<double> zero_vdot =
        VectorX<double>::Zero(plant_->num_velocities());
    // Zero velocities.
    shoulder_->set_angular_rate(plant_context_, 0.0);
    elbow_->set_angular_rate(plant_context_, 0.0);
    const VectorX<double> tau_g_id =
        -plant_->CalcInverseDynamics(*plant_context_, zero_vdot, forces);
    EXPECT_TRUE(CompareMatrices(tau_g_id, tau_g_expected, kTolerance,
                                MatrixCompareType::relative));
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

    // Verify that the implicit dynamics match the continuous ones.
    VectorXd residual = diagram_->AllocateImplicitTimeDerivativesResidual();
    diagram_->CalcImplicitTimeDerivativesResidual(*context_, *derivatives_,
                                                  &residual);
    EXPECT_TRUE(CompareMatrices(residual, VectorXd::Zero(4), 1e-14));
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
  DRAKE_EXPECT_NO_THROW(poses_value->get_value<FramePoseVector<double>>());
  const FramePoseVector<double>& poses =
      poses_value->get_value<FramePoseVector<double>>();

  // Compute the poses for each geometry in the model.
  plant_->get_geometry_poses_output_port().Calc(*context, poses_value.get());
  EXPECT_EQ(poses.size(), 2);  // Only two frames move.

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
    const std::optional<FrameId> optional_id =
        plant_->GetBodyFrameIdIfExists(body_index);
    ASSERT_TRUE(optional_id.has_value());
    EXPECT_EQ(frame_id, *optional_id);
    EXPECT_EQ(body_index, plant_->GetBodyFromFrameId(frame_id)->index());
    const RigidTransform<double>& X_WB = poses.value(frame_id);
    const RigidTransform<double>& X_WB_expected =
        plant_->EvalBodyPoseInWorld(*context, plant_->get_body(body_index));
    EXPECT_TRUE(CompareMatrices(X_WB.GetAsMatrix34(),
                                X_WB_expected.GetAsMatrix34(),
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
  std::optional<FrameId> undefined_id =
      plant_->GetBodyFrameIdIfExists(world_index());
  EXPECT_EQ(undefined_id, std::nullopt);
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

GTEST_TEST(MultibodyPlantTest, SetDefaultFreeBodyPose) {
  // We cannot use Acrobot for testing `SetDefaultFreeBodyPose` since it has no
  // free bodies.
  MultibodyPlant<double> plant(0.0);
  const auto& body = plant.AddRigidBody("body", SpatialInertia<double>());
  EXPECT_TRUE(CompareMatrices(
      plant.GetDefaultFreeBodyPose(body).GetAsMatrix4(),
      RigidTransformd::Identity().GetAsMatrix4()));
  const RigidTransformd X_WB_default(
      RollPitchYawd(0.1, 0.2, 0.3), Vector3d(1, 2, 3));
  plant.SetDefaultFreeBodyPose(body, X_WB_default);
  EXPECT_TRUE(CompareMatrices(
      plant.GetDefaultFreeBodyPose(body).GetAsMatrix4(),
      X_WB_default.GetAsMatrix4()));
  plant.Finalize();
  EXPECT_GT(plant.num_positions(), 0);
  auto context = plant.CreateDefaultContext();
  const double kTolerance = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(
      body.EvalPoseInWorld(*context).GetAsMatrix4(),
      X_WB_default.GetAsMatrix4(), kTolerance));
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

// A basic sanity check for context cloning.
TEST_F(AcrobotPlantTests, ContextClone) {
  shoulder_->set_default_angle(0.05);
  auto old_context = plant_->CreateDefaultContext();
  auto new_context = old_context->Clone();
  shoulder_->set_angle(old_context.get(), 0.01);
  EXPECT_EQ(shoulder_->get_angle(*old_context), 0.01);
  EXPECT_EQ(shoulder_->get_angle(*new_context), 0.05);
}

GTEST_TEST(MultibodyPlantTest, Graphviz) {
  MultibodyPlant<double> plant(0.0);
  const std::string acrobot_path =
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.sdf");
  const std::string cylinder_path = FindResourceOrThrow(
      "drake/multibody/benchmarks/free_body/uniform_solid_cylinder.urdf");
  Parser(&plant).AddModelFromFile(acrobot_path);
  Parser(&plant).AddModelFromFile(cylinder_path);
  Parser(&plant).AddModelFromFile(cylinder_path, "cylinder2");

  plant.set_name("MyTestMBP");
  const std::string dot = plant.GetTopologyGraphvizString();

  // Check that the diagram is labeled with the system name.
  EXPECT_NE(std::string::npos, dot.find("MyTestMBP")) << dot;
  // Check that we have subgraphs and they use the "cluster" prefix.
  EXPECT_NE(std::string::npos, dot.find("subgraph cluster")) << dot;
  // Check that we have a body0 - body4 (world, 2 for acrobot, 2 cylinders).
  for (int i = 0; i < 5; ++i) {
    EXPECT_NE(std::string::npos, dot.find("body" + std::to_string(i))) << dot;
  }
  // Check that the cylinder's body appears twice.
  const size_t pos = dot.find("uniformSolidCylinder");
  EXPECT_NE(std::string::npos, pos) << dot;
  EXPECT_NE(std::string::npos, dot.find("uniformSolidCylinder", pos + 1))
      << dot;
  // Check for the second cylinder model instance.
  EXPECT_NE(std::string::npos, dot.find("cylinder2")) << dot;
  // Check for the Acrobot elbow joint.
  EXPECT_NE(std::string::npos, dot.find("ElbowJoint [revolute]")) << dot;

  // Check that the same string appears before and after calling Finalize().
  plant.Finalize();
  EXPECT_STREQ(dot.c_str(), plant.GetTopologyGraphvizString().c_str());
}

// Verifies that the right errors get invoked upon finalization.
GTEST_TEST(MultibodyPlantTest, FilterAdjacentBodiesSourceErrors) {
  SceneGraph<double> scene_graph;

  // Case: Finalize w/o having registered as geometry source but without
  // providing a scene graph -- no error.
  {
    MultibodyPlant<double> plant(0.0);
    DRAKE_EXPECT_NO_THROW(plant.Finalize());
  }

  // Case: Registered as source, correct finalization.
  {
    MultibodyPlant<double> plant(0.0);
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    DRAKE_EXPECT_NO_THROW(plant.Finalize());
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
// The chain terminates with one additional body with no geometry.  It has no
// bearing on collision tests but is used for geometry collection testing.
//
// Also accepts a filtering function that is applied between geometry
// registration and context compilation.
class SphereChainScenario {
 public:
  SphereChainScenario(
      int sphere_count,
      std::function<void(SphereChainScenario*)> apply_filters = nullptr,
      bool finalize = true)
      : sphere_count_(sphere_count),
        apply_filters_(apply_filters) {
    using std::to_string;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder_, 0.0);

    // A half-space for the ground geometry.
    ground_id_ = plant_->RegisterCollisionGeometry(
        plant_->world_body(),
        // A half-space passing through the origin in the x-z plane.
        geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero()),
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
    for (int i = 0; i < sphere_count_; ++i) {
      // TODO(SeanCurtis-TRI): Make this prettier when C++17 is available.
      // E.g., auto [id, geometry] = make_sphere(i);
      GeometryId id{};
      const RigidBody<double>* body{};
      std::tie(body, id) = make_sphere(i);
      spheres_.push_back(body);
      sphere_ids_.push_back(id);
    }
    // Add hinges between spheres.
    for (int i = 0; i < sphere_count_ - 1; ++i) {
      plant_->AddJoint<RevoluteJoint>(
          "hinge" + to_string(i) + "_" + to_string(i + 1), *spheres_[i],
          std::nullopt, *spheres_[i + 1], std::nullopt, Vector3d::UnitY());
    }

    // Body with no registered frame.
    no_geometry_body_ = &plant_->AddRigidBody("NothingRegistered",
                                              SpatialInertia<double>());

    if (finalize) {
      Finalize();
    }
  }

  void Finalize() {
    // We are done defining the model.
    plant_->Finalize();

    if (apply_filters_) apply_filters_(this);

    diagram_ = builder_.Build();
    context_ = diagram_->CreateDefaultContext();

    // Set the zero configuration.
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, context_.get());

    // NOTE: Only ids for collision geometries are included.
    for (int i = 0; i < sphere_count_; ++i) {
      unfiltered_collisions_.insert(std::make_pair(ground_id(), sphere_id(i)));
      for (int j = i + 2; j < sphere_count_; ++j) {
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

  // Get all bodies of the internal plant.
  std::vector<const Body<double>*> get_all_bodies() const {
    std::vector<const Body<double>*> all_bodies;
    all_bodies.push_back(no_geometry_body_);
    for (const auto sphere : spheres_) {
      all_bodies.push_back(sphere);
    }
    all_bodies.push_back(&plant_->world_body());
    return all_bodies;
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
  // For plant and scene graph construction.
  int sphere_count_{};
  systems::DiagramBuilder<double> builder_;
  std::function<void(SphereChainScenario*)> apply_filters_;

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

// This confirms that every body *always* has a corresponding SceneGraph FrameId
// if the MBP has been registered as a SceneGraph source.
GTEST_TEST(MultibodyPlantTest, AutoBodySceneGraphRegistration) {
  MultibodyPlant<double> plant(0.0);
  const RigidBody<double>& body1 = plant.AddRigidBody(
      "body1", SpatialInertia<double>());
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetBodyFrameIdOrThrow(body1.index()), std::logic_error,
      "Body 'body1' does not have geometry registered with it.");

  geometry::SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  ASSERT_TRUE(plant.geometry_source_is_registered());

  // Now that the plant has been registered as a source, old bodies have been
  // updated with FrameIds.
  DRAKE_EXPECT_NO_THROW(plant.GetBodyFrameIdOrThrow(body1.index()));

  // And new bodies have FrameIds immediately upon creation.
  const RigidBody<double>& body2 = plant.AddRigidBody(
      "body2", SpatialInertia<double>());
  DRAKE_EXPECT_NO_THROW(plant.GetBodyFrameIdOrThrow(body2.index()));
}

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
  MultibodyPlant<double> plant(0.0);

  // A throw-away rigid body I can use to satisfy the function interface; it
  // will never be used because the function will fail in a pre-requisite test.
  RigidBody<double> body{SpatialInertia<double>()};
  // The case where the plant has *not* been finalized.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.CollectRegisteredGeometries({&body}), std::runtime_error,
      "Failure .* in CollectRegisteredGeometries.* failed.");

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

  const bool finalize = false;
  SphereChainScenario scenario(5, nullptr, finalize);

  const MultibodyPlant<double>& plant = *scenario.mutable_plant();

  auto check_geometries = [&]() {
    // Case: Empty vector produces empty geometry set.
    {
      GeometrySet set = plant.CollectRegisteredGeometries({});
      GeometrySetTester tester(&set);
      EXPECT_EQ(tester.num_geometries(), 0);
      EXPECT_EQ(tester.num_frames(), 0);
    }

    // Case: Single body produces single, corresponding frame.
    {
      GeometrySet set =
          plant.CollectRegisteredGeometries({&scenario.sphere(0)});
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
      EXPECT_EQ(tester.num_frames(), 1);
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

    // Case: Consider all bodies.
    {
      GeometrySet set =
          plant.CollectRegisteredGeometries(scenario.get_all_bodies());
      GeometrySetTester tester(&set);
      EXPECT_EQ(tester.num_frames(), plant.num_bodies());
      EXPECT_EQ(tester.num_geometries(), 0);
      EXPECT_FALSE(tester.contains(scenario.ground_id()));
    }
  };

  {
    SCOPED_TRACE("pre-finalize");
    EXPECT_FALSE(plant.is_finalized());
    check_geometries();
  }

  {
    SCOPED_TRACE("post-finalize");
    scenario.Finalize();
    EXPECT_TRUE(plant.is_finalized());
    check_geometries();
  }
}

// Verifies the process of getting welded bodies.
GTEST_TEST(MultibodyPlantTest, GetBodiesWeldedTo) {
  using ::testing::UnorderedElementsAre;
  // This test expects that the following model has a world body and a pair of
  // welded-together bodies.
  const std::string sdf_file =
      FindResourceOrThrow("drake/multibody/plant/test/split_pendulum.sdf");
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelFromFile(sdf_file);
  const Body<double>& upper = plant.GetBodyByName("upper_section");
  const Body<double>& lower = plant.GetBodyByName("lower_section");

  // Add a new body, and weld it using `WeldFrames` (to ensure that topology is
  // updated via this API).
  const Body<double>& extra = plant.AddRigidBody(
      "extra", default_model_instance(), SpatialInertia<double>());
  plant.WeldFrames(plant.world_frame(), extra.body_frame());

  // Verify we can call GetBodiesWeldedTo() pre-finalize.
  EXPECT_THAT(plant.GetBodiesWeldedTo(plant.world_body()),
              UnorderedElementsAre(&plant.world_body(), &extra));
  EXPECT_THAT(plant.GetBodiesWeldedTo(lower),
              UnorderedElementsAre(&upper, &lower));

  // And post-finalize.
  plant.Finalize();
  EXPECT_THAT(plant.GetBodiesWeldedTo(plant.world_body()),
              UnorderedElementsAre(&plant.world_body(), &extra));
  EXPECT_THAT(plant.GetBodiesWeldedTo(lower),
              UnorderedElementsAre(&upper, &lower));

  // Briefly test scalar conversion.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);
  const Body<AutoDiffXd>& upper_ad = plant_ad->GetBodyByName("upper_section");
  const Body<AutoDiffXd>& lower_ad = plant_ad->GetBodyByName("lower_section");
  const Body<AutoDiffXd>& extra_ad = plant_ad->GetBodyByName("extra");

  EXPECT_THAT(plant_ad->GetBodiesWeldedTo(plant_ad->world_body()),
              UnorderedElementsAre(&plant_ad->world_body(), &extra_ad));
  EXPECT_THAT(plant_ad->GetBodiesWeldedTo(lower_ad),
              UnorderedElementsAre(&upper_ad, &lower_ad));
}

// Regression test for unhelpful error message -- see #14641.
GTEST_TEST(MultibodyPlantTest, ReversedWeldError) {
  // This test expects that the following model has a world body and a pair of
  // welded-together bodies.
  const std::string sdf_file =
      FindResourceOrThrow("drake/multibody/plant/test/split_pendulum.sdf");
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelFromFile(sdf_file);

  // Add a new body, and weld it in the wrong direction using `WeldFrames`.
  const Body<double>& extra = plant.AddRigidBody(
      "extra", default_model_instance(), SpatialInertia<double>());
  plant.WeldFrames(extra.body_frame(), plant.world_frame());

  // The important property of this message is that it reports some identifier
  // for the involved objects, so at least the developer can map those back to
  // objects and deduce what API call was in error. If the details of the
  // message change, update this check to match. If in future the error can be
  // caught at the WeldFrames() step, so much the better. Modify this test to
  // reflect that.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.Finalize(),
      std::runtime_error,
      "This multibody tree already has a mobilizer connecting "
      "inboard frame \\(index=0\\) and outboard frame \\(index=\\d*\\). "
      "More than one mobilizer between two frames is not allowed.");
}

// Utility to verify that the only ports of MultibodyPlant that are feedthrough
// are acceleration and reaction force ports.
bool OnlyAccelerationAndReactionPortsFeedthrough(
    const MultibodyPlant<double>& plant) {
  // Create a set of the indices of all ports that can be feedthrough.
  std::set<int> ok_to_feedthrough;
  ok_to_feedthrough.insert(plant.get_reaction_forces_output_port().get_index());
  ok_to_feedthrough.insert(
      plant.get_generalized_acceleration_output_port().get_index());
  for (ModelInstanceIndex i(0); i < plant.num_model_instances(); ++i)
    ok_to_feedthrough.insert(
        plant.get_generalized_acceleration_output_port(i).get_index());
  ok_to_feedthrough.insert(
      plant.get_body_spatial_accelerations_output_port().get_index());

  // Now find all the feedthrough ports and make sure they are on the
  // list of expected feedthrough ports.
  const std::multimap<int, int> feedthroughs = plant.GetDirectFeedthroughs();
  for (const auto& inout_pair : feedthroughs) {
    if (ok_to_feedthrough.count(inout_pair.second) == 0)
      return false;  // Found a spurious feedthrough port.
  }
  return true;
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
  MultibodyPlant<double> plant(0.0);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // A half-space for the ground geometry.
  CoulombFriction<double> ground_friction(0.5, 0.3);
  GeometryId ground_id = plant.RegisterCollisionGeometry(
      plant.world_body(),
      // A half-space passing through the origin in the x-z plane.
      geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero()),
      geometry::HalfSpace(), "ground", ground_friction);

  // Add two spherical bodies.
  const RigidBody<double>& sphere1 =
      plant.AddRigidBody("Sphere1", SpatialInertia<double>());
  CoulombFriction<double> sphere1_friction(0.8, 0.5);
  // estimated parameters for mass=1kg, penetration_tolerance=0.01m
  // and gravity g=9.8 m/s^2.
  const double sphere1_stiffness = 980;    // N/m
  const double sphere1_dissipation = 3.2;  // s/m
  geometry::ProximityProperties sphere1_properties;
  sphere1_properties.AddProperty(geometry::internal::kMaterialGroup,
                                 geometry::internal::kFriction,
                                 sphere1_friction);
  sphere1_properties.AddProperty(geometry::internal::kMaterialGroup,
                                 geometry::internal::kPointStiffness,
                                 sphere1_stiffness);
  sphere1_properties.AddProperty(geometry::internal::kMaterialGroup,
                                 geometry::internal::kHcDissipation,
                                 sphere1_dissipation);
  GeometryId sphere1_id = plant.RegisterCollisionGeometry(
      sphere1, RigidTransformd::Identity(), geometry::Sphere(radius),
      "collision", std::move(sphere1_properties));

  geometry::ProximityProperties sphere2_properties;
  sphere2_properties.AddProperty("test", "dummy", 7);
  CoulombFriction<double> sphere2_friction(0.7, 0.6);
  // estimated parameters for mass=1kg, penetration_tolerance=0.05m
  // and gravity g=9.8 m/s^2.
  const double sphere2_stiffness = 196;    // N/m
  const double sphere2_dissipation = 1.4;  // s/m
  sphere2_properties.AddProperty(geometry::internal::kMaterialGroup,
                                 geometry::internal::kFriction,
                                 sphere2_friction);
  sphere2_properties.AddProperty(geometry::internal::kMaterialGroup,
                                 geometry::internal::kPointStiffness,
                                 sphere2_stiffness);
  sphere2_properties.AddProperty(geometry::internal::kMaterialGroup,
                                 geometry::internal::kHcDissipation,
                                 sphere2_dissipation);
  const RigidBody<double>& sphere2 =
      plant.AddRigidBody("Sphere2", SpatialInertia<double>());
  GeometryId sphere2_id = plant.RegisterCollisionGeometry(
      sphere2, RigidTransformd::Identity(), geometry::Sphere(radius),
      "collision", std::move(sphere2_properties));

  // Confirm externally-defined proximity properties propagate through.
  {
    EXPECT_NE(scene_graph.model_inspector().GetProximityProperties(sphere2_id),
              nullptr);
    const geometry::ProximityProperties& props =
        *scene_graph.model_inspector().GetProximityProperties(sphere2_id);
    EXPECT_TRUE(props.HasProperty("test", "dummy"));
    EXPECT_EQ(props.GetProperty<int>("test", "dummy"), 7);
  }

  // We are done defining the model.
  plant.Finalize();

  // Only accelerations and joint reaction forces feedthrough, even with the
  // new ports related to SceneGraph interaction.
  EXPECT_TRUE(OnlyAccelerationAndReactionPortsFeedthrough(plant));

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
      RigidTransformd(Vector3d(x_offset, radius, 0.0)));

  unique_ptr<AbstractValue> poses_value =
      plant.get_geometry_poses_output_port().Allocate();
  DRAKE_EXPECT_NO_THROW(poses_value->get_value<FramePoseVector<double>>());
  const FramePoseVector<double>& pose_data =
      poses_value->get_value<FramePoseVector<double>>();

  // Compute the poses for each geometry in the model.
  plant.get_geometry_poses_output_port().Calc(*context, poses_value.get());
  EXPECT_EQ(pose_data.size(), 2);  // Only two frames move.

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  for (BodyIndex body_index(1);
       body_index < plant.num_bodies(); ++body_index) {
    const FrameId frame_id = plant.GetBodyFrameIdOrThrow(body_index);
    const RigidTransform<double>& X_WB = pose_data.value(frame_id);
    const RigidTransform<double>& X_WB_expected =
        plant.EvalBodyPoseInWorld(*context, plant.get_body(body_index));
    EXPECT_TRUE(CompareMatrices(X_WB.GetAsMatrix34(),
                                X_WB_expected.GetAsMatrix34(),
                                kTolerance, MatrixCompareType::relative));
  }

  // Verify we can retrieve friction coefficients, propagated through to SG.
  const geometry::ProximityProperties& ground_props =
      *scene_graph.model_inspector().GetProximityProperties(ground_id);
  const geometry::ProximityProperties& sphere1_props =
      *scene_graph.model_inspector().GetProximityProperties(sphere1_id);
  const geometry::ProximityProperties& sphere2_props =
      *scene_graph.model_inspector().GetProximityProperties(sphere2_id);

  EXPECT_TRUE(ground_props.GetProperty<CoulombFriction<double>>(
                  geometry::internal::kMaterialGroup,
                  geometry::internal::kFriction) == ground_friction);
  EXPECT_TRUE(sphere1_props.GetProperty<CoulombFriction<double>>(
                  geometry::internal::kMaterialGroup,
                  geometry::internal::kFriction) == sphere1_friction);
  EXPECT_TRUE(sphere2_props.GetProperty<CoulombFriction<double>>(
                  geometry::internal::kMaterialGroup,
                  geometry::internal::kFriction) == sphere2_friction);

  EXPECT_TRUE(sphere1_props.GetProperty<double>(
                  geometry::internal::kMaterialGroup,
                  geometry::internal::kPointStiffness) == sphere1_stiffness);
  EXPECT_TRUE(sphere2_props.GetProperty<double>(
                  geometry::internal::kMaterialGroup,
                  geometry::internal::kPointStiffness) == sphere2_stiffness);

  EXPECT_TRUE(sphere1_props.GetProperty<double>(
                  geometry::internal::kMaterialGroup,
                  geometry::internal::kHcDissipation) == sphere1_dissipation);
  EXPECT_TRUE(sphere2_props.GetProperty<double>(
                  geometry::internal::kMaterialGroup,
                  geometry::internal::kHcDissipation) == sphere2_dissipation);
}

// Verifies the process of visual geometry registration with a SceneGraph.
// We build a model with two spheres and a ground plane. The ground plane is
// located at y = 0 with normal in the y-axis direction.
GTEST_TEST(MultibodyPlantTest, VisualGeometryRegistration) {
  // Parameters of the setup.
  const double radius = 0.5;

  SceneGraph<double> scene_graph;

  // Add a render engine so we can confirm that the current default behavior of
  // assigning perception roles to visual geometries is happening.
  auto temp_engine = std::make_unique<DummyRenderEngine>();
  // We want to confirm MBP is assigning perception roles; so we force the
  // engine to accept all registered visuals so we don't have to worry about
  // possible *conditions* of acceptance. To attempt a registration is to
  // succeed. That way, we can simply look at the number of registered ids to
  // determine role assignment has happened.
  temp_engine->set_force_accept(true);
  const DummyRenderEngine& render_engine = *temp_engine;
  scene_graph.AddRenderer("dummy", move(temp_engine));
  MultibodyPlant<double> plant(0.0);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  EXPECT_EQ(render_engine.num_registered(), 0);

  // A half-space for the ground geometry -- uses default visual material
  GeometryId ground_id = plant.RegisterVisualGeometry(
      plant.world_body(),
      // A half-space passing through the origin in the x-z plane.
      geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero()),
      geometry::HalfSpace(), "ground");
  EXPECT_EQ(render_engine.num_registered(), 1);

  // Add two spherical bodies.
  const RigidBody<double>& sphere1 =
      plant.AddRigidBody("Sphere1", SpatialInertia<double>());
  Vector4<double> sphere1_diffuse{0.9, 0.1, 0.1, 0.5};
  GeometryId sphere1_id = plant.RegisterVisualGeometry(
      sphere1, RigidTransformd::Identity(), geometry::Sphere(radius),
      "visual", sphere1_diffuse);
  EXPECT_EQ(render_engine.num_registered(), 2);
  const RigidBody<double>& sphere2 =
      plant.AddRigidBody("Sphere2", SpatialInertia<double>());
  IllustrationProperties sphere2_props;
  const Vector4<double> sphere2_diffuse{0.1, 0.9, 0.1, 0.5};
  sphere2_props.AddProperty("phong", "diffuse", sphere2_diffuse);
  sphere2_props.AddProperty("phong", "diffuse_map", "empty.png");
  sphere2_props.AddProperty("renderer", "accepting",
                            std::set<std::string>{"not_dummy"});
  GeometryId sphere2_id = plant.RegisterVisualGeometry(
      sphere2, RigidTransformd::Identity(), geometry::Sphere(radius), "visual",
      sphere2_props);
  // Because sphere 2 white listed a *different* renderer, it didn't get added
  // to render_engine.
  EXPECT_EQ(render_engine.num_registered(), 2);

  // We are done defining the model.
  plant.Finalize();

  EXPECT_EQ(plant.num_visual_geometries(), 3);
  EXPECT_EQ(plant.num_collision_geometries(), 0);
  EXPECT_TRUE(plant.geometry_source_is_registered());
  EXPECT_TRUE(plant.get_source_id());

  unique_ptr<Context<double>> context = scene_graph.CreateDefaultContext();
  unique_ptr<AbstractValue> state_value =
      scene_graph.get_query_output_port().Allocate();
  DRAKE_EXPECT_NO_THROW(state_value->get_value<QueryObject<double>>());
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
    const IllustrationProperties* material =
        inspector.GetIllustrationProperties(sphere2_id);
    ASSERT_TRUE(material->HasProperty("phong", "diffuse_map"));
    EXPECT_EQ(material->GetProperty<std::string>("phong", "diffuse_map"),
        "empty.png");
  }
}

GTEST_TEST(MultibodyPlantTest, AutoDiffCalcPointPairPenetrations) {
  PendulumParameters parameters;
  unique_ptr<MultibodyPlant<double>> pendulum = MakePendulumPlant(parameters);
  unique_ptr<Context<double>> context = pendulum->CreateDefaultContext();

  // We connect a SceneGraph to the pendulum plant in order to enforce the
  // creation of geometry input/output ports. This ensures the call to
  // CalcPointPairPenetrations evaluates appropriately.
  geometry::SceneGraph<double> scene_graph;
  pendulum->RegisterAsSourceForSceneGraph(&scene_graph);

  auto autodiff_pendulum =
      drake::systems::System<double>::ToAutoDiffXd(*pendulum.get());
  auto autodiff_context = autodiff_pendulum->CreateDefaultContext();

  // This test case contains no collisions, and hence we should not throw.
  DRAKE_EXPECT_NO_THROW(
      autodiff_pendulum->EvalPointPairPenetrations(*autodiff_context.get()));
}

GTEST_TEST(MultibodyPlantTest, CalcPointPairPenetrationsDisconnectedPorts) {
  // Creates a plant and register geometry with a SceneGraph, but does not
  // connect their respective ports in a Diagram. MultibodyPlant will know
  // that it is registered as a source for geometry, but will fail to Eval
  // its geometry_query_input_port(). Check that this failure happens as
  // expected.
  SceneGraph<double> scene_graph;
  MultibodyPlant<double> plant(0.0);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  plant.Finalize();
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  // Plant was not connected to the SceneGraph in a diagram, so its input port
  // should be invalid.
  DRAKE_EXPECT_THROWS_MESSAGE(
      MultibodyPlantTester::EvalGeometryQueryInput(plant, *context),
      std::logic_error,
      "The geometry query input port \\(see "
      "MultibodyPlant::get_geometry_query_input_port\\(\\)\\) "
      "of this MultibodyPlant is not connected. Please connect the"
      "geometry query output port of a SceneGraph object "
      "\\(see SceneGraph::get_query_output_port\\(\\)\\) to this plants input "
      "port in a Diagram.");
}

GTEST_TEST(MultibodyPlantTest, LinearizePendulum) {
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

  PendulumParameters parameters;
  unique_ptr<MultibodyPlant<double>> pendulum = MakePendulumPlant(parameters);
  const auto& pin =
      pendulum->GetJointByName<RevoluteJoint>(parameters.pin_joint_name());
  unique_ptr<Context<double>> context = pendulum->CreateDefaultContext();
  pendulum->get_actuation_input_port().FixValue(context.get(), 0.0);
  pendulum->get_applied_generalized_force_input_port().FixValue(context.get(),
                                                                0.0);

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

TEST_F(AcrobotPlantTests, EvalStateAndAccelerationOutputPorts) {
  EXPECT_EQ(plant_->num_visual_geometries(), 3);
  EXPECT_TRUE(plant_->geometry_source_is_registered());
  EXPECT_TRUE(plant_->get_source_id());

  // The default context gets initialized by a call to SetDefaultState(), which
  // for a MultibodyPlant sets all revolute joints to have zero angles and zero
  // angular velocity.
  unique_ptr<systems::Context<double>> diagram_context =
      diagram_->CreateDefaultContext();
  Context<double>& context =
      plant_->GetMyMutableContextFromRoot(diagram_context.get());

  // Set some non-zero state:
  shoulder_->set_angle(&context, M_PI / 3.0);
  elbow_->set_angle(&context, -0.2);
  shoulder_->set_angular_rate(&context, -0.5);
  elbow_->set_angular_rate(&context, 2.5);

  unique_ptr<AbstractValue> state_value =
      plant_->get_state_output_port().Allocate();
  DRAKE_EXPECT_NO_THROW(state_value->get_value<BasicVector<double>>());
  const BasicVector<double>& state_out =
      state_value->get_value<BasicVector<double>>();
  EXPECT_EQ(state_out.size(), plant_->num_multibody_states());

  // Compute the poses for each geometry in the model.
  plant_->get_state_output_port().Calc(context, state_value.get());

  // Get continuous state_out from context.
  const VectorBase<double>& state = context.get_continuous_state_vector();

  // Verify state_out indeed matches state.
  EXPECT_EQ(state_out.CopyToVector(), state.CopyToVector());

  // Now calculate accelerations and make sure they show up on the
  // all-vdot port and on the appropriate model instance port.

  plant_->get_actuation_input_port().FixValue(&context, 0.0);
  // Time derivatives includes both qdot and vdot.
  const auto& derivs = plant_->EvalTimeDerivatives(context);
  const auto& vdot = derivs.get_generalized_velocity();
  EXPECT_EQ(vdot.size(), plant_->num_velocities());
  const auto& accel = plant_->get_generalized_acceleration_output_port()
      .Eval<BasicVector<double>>(context);
  EXPECT_EQ(accel.size(), plant_->num_velocities());
  EXPECT_EQ(accel.CopyToVector(), vdot.CopyToVector());

  // All the elements should be in the same model instance, so just ask one.
  const ModelInstanceIndex instance = shoulder_->model_instance();
  const auto& accel_instance =
      plant_->get_generalized_acceleration_output_port(instance)
      .Eval<BasicVector<double>>(context);
  EXPECT_EQ(accel_instance.size(), plant_->num_velocities());
  EXPECT_EQ(accel_instance.CopyToVector(), vdot.CopyToVector());

  // Check that unused model instance ports are present and produce 0-length
  // results.
  const auto& accel_default_instance =
      plant_->get_generalized_acceleration_output_port(default_model_instance())
          .Eval<BasicVector<double>>(context);
  EXPECT_EQ(accel_default_instance.size(), 0);

  const auto& state_world_instance =
      plant_->get_state_output_port(world_model_instance())
          .Eval<BasicVector<double>>(context);
  EXPECT_EQ(state_world_instance.size(), 0);
}

// Helper function for the two v-to-qdot and qdot-to-v tests.
void InitializePlantAndContextForVelocityToQDotMapping(
    MultibodyPlant<double>* plant, std::unique_ptr<Context<double>>* context) {
  // This is used in purely kinematic tests. Therefore we leave the spatial
  // inertia initialized to garbage. It should not affect the results.
  const RigidBody<double>& body =
      plant->AddRigidBody("FreeBody", SpatialInertia<double>());
  plant->Finalize();

  *context = plant->CreateDefaultContext();

  // Set an arbitrary pose of the body in the world.
  const Vector3d p_WB(1, 2, 3);  // Position in world.
  const Vector3d axis_W =        // Orientation in world.
      (1.5 * Vector3d::UnitX() +
       2.0 * Vector3d::UnitY() +
       3.0 * Vector3d::UnitZ()).normalized();
  const math::RigidTransformd X_WB(AngleAxisd(M_PI / 3.0, axis_W), p_WB);
  plant->SetFreeBodyPose(context->get(), body, X_WB);

  // Set an arbitrary, non-zero, spatial velocity of B in W.
  const SpatialVelocity<double> V_WB(Vector3d(1.0, 2.0, 3.0),
                                     Vector3d(-1.0, 4.0, -0.5));
  plant->SetFreeBodySpatialVelocity(context->get(), body, V_WB);
}

// Tests the qdot-to-v mapping when all objects in the world are fixed (have
// no degrees-of-freedom).
GTEST_TEST(MultibodyPlantTest, MapVelocityToQDotAndBackFixedWorld) {
  MultibodyPlant<double> plant(0.0);
  test::AddFixedObjectsToPlant(&plant);
  plant.Finalize();
  unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  // Make sure that the mapping functions do not throw.
  BasicVector<double> qdot(0), v(0);
  ASSERT_NO_THROW(plant.MapVelocityToQDot(*context, v, &qdot));
  ASSERT_NO_THROW(plant.MapQDotToVelocity(*context, qdot, &v));
}

GTEST_TEST(MultibodyPlantTest, MapVelocityToQDotAndBackContinuous) {
  MultibodyPlant<double> plant(0.0);
  unique_ptr<Context<double>> context;
  InitializePlantAndContextForVelocityToQDotMapping(&plant, &context);

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

GTEST_TEST(MultibodyPlantTest, MapVelocityToQDotAndBackDiscrete) {
  const double time_step = 1e-3;
  MultibodyPlant<double> plant(time_step);
  unique_ptr<Context<double>> context;
  InitializePlantAndContextForVelocityToQDotMapping(&plant, &context);

  // Use of MultibodyPlant's mapping to convert generalized velocities to time
  // derivatives of generalized coordinates.
  BasicVector<double> qdot(plant.num_positions());
  BasicVector<double> v(plant.num_velocities());
  ASSERT_EQ(qdot.size(), 7);
  ASSERT_EQ(v.size(), 6);
  v.SetFromVector(plant.GetVelocities(*context));
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
  MultibodyPlant<double> plant_{0.0};
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

// Verify that we can obtain the owning MultibodyPlant from one of its
// MultibodyElements, and that we get a proper error message if we try
// this for an element that isn't owned by a MultibodyPlant.
TEST_F(SplitPendulum, GetMultibodyPlantFromElement) {
  const MultibodyPlant<double>& pins_plant = pin_->GetParentPlant();
  EXPECT_EQ(&pins_plant, &plant_);

  // Create an element-owning MBTreeSystem that _is not_ an MBPlant.
  struct MyMBSystem : public internal::MultibodyTreeSystem<double> {
    MyMBSystem() {
      rigid_body = &mutable_tree().AddBody<RigidBody>(SpatialInertia<double>());
      Finalize();
    }
    const RigidBody<double>* rigid_body{};
  } mb_system;

  DRAKE_EXPECT_THROWS_MESSAGE(
      mb_system.rigid_body->GetParentPlant(), std::logic_error,
      ".*multibody element.*not owned by.*MultibodyPlant.*");
}

// Verifies we can parse link collision geometries and surface friction.
GTEST_TEST(MultibodyPlantTest, ScalarConversionConstructor) {
  const std::string full_name = drake::FindResourceOrThrow(
      "drake/multibody/parsing/test/"
          "links_with_visuals_and_collisions.sdf");
  MultibodyPlant<double> plant(0.0);
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

  // Make sure the geometry ports were included in the autodiffed plant.
  DRAKE_EXPECT_NO_THROW(plant_autodiff.get_geometry_query_input_port());
  DRAKE_EXPECT_NO_THROW(plant_autodiff.get_geometry_poses_output_port());
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

    plant_.SetFreeBodyPose(context, large_box, X_WLb);
    plant_.SetFreeBodyPose(context, small_box, X_WSb);
  }

  // Generate a valid set of penetrations for this particular setup that
  // emulates a multicontact scenario.
  void SetPenetrationPairs(
      const Context<double>& context,
      std::vector<PenetrationAsPointPair<double>>* penetrations) {
    const Body<double>& large_box = plant_.GetBodyByName("LargeBox");
    const Body<double>& small_box = plant_.GetBodyByName("SmallBox");

    // Pose of the boxes in the world frame.
    const RigidTransform<double>& X_WLb =
        plant_.EvalBodyPoseInWorld(context, large_box);
    const RigidTransform<double>& X_WSb =
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
    math::InitializeAutoDiff(v, &v_autodiff);
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
      const RigidTransform<T>& X_WA = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyA_index));
      const SpatialVelocity<T> V_WA =
          plant_on_T.EvalBodySpatialVelocityInWorld(
              context_on_T, plant_on_T.get_body(bodyA_index));

      BodyIndex bodyB_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_B);
      const RigidTransform<T>& X_WB = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyB_index));
      const SpatialVelocity<T> V_WB =
          plant_on_T.EvalBodySpatialVelocityInWorld(
              context_on_T, plant_on_T.get_body(bodyB_index));

      const Vector3<T> p_WCa = pair.p_WCa.cast<T>();
      const Vector3<T> p_WCb = pair.p_WCb.cast<T>();
      // The one and only point of contact C.
      // Thus far MBP places it midway for point contact.
      const Vector3<T> p_WC = 0.5 * (p_WCa + p_WCb);

      const Vector3<T> p_WAo = X_WA.translation();
      const Vector3<T> p_AoCa_W = p_WC - p_WAo;
      const Vector3<T> v_WCa = V_WA.Shift(p_AoCa_W).translational();

      const Vector3<T> p_WBo = X_WB.translation();
      const Vector3<T> p_BoCb_W = p_WC - p_WBo;
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
      const RigidTransform<T>& X_WA = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyA_index));
      const SpatialVelocity<T> V_WA =
          plant_on_T.EvalBodySpatialVelocityInWorld(
              context_on_T, plant_on_T.get_body(bodyA_index));

      BodyIndex bodyB_index = MultibodyPlantTester::geometry_id_to_body_index(
          plant_on_T, pair.id_B);
      const RigidTransform<T>& X_WB = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyB_index));
      const SpatialVelocity<T> V_WB =
          plant_on_T.EvalBodySpatialVelocityInWorld(
              context_on_T, plant_on_T.get_body(bodyB_index));

      const Vector3<T> p_WCa = pair.p_WCa.cast<T>();
      const Vector3<T> p_WCb = pair.p_WCb.cast<T>();
      // The one and only point of contact C.
      // Thus far MBP places it midway for point contact.
      const Vector3<T> p_WC = 0.5 * (p_WCa + p_WCb);

      const Vector3<T> p_WAo = X_WA.translation();
      const Vector3<T> p_AoCa_W = p_WC - p_WAo;
      const Vector3<T> v_WCa = V_WA.Shift(p_AoCa_W).translational();

      const Vector3<T> p_WBo = X_WB.translation();
      const Vector3<T> p_BoCb_W = p_WC - p_WBo;
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
  MultibodyPlant<double> plant_{0.0};
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
  const MatrixX<double> vn_derivs = math::ExtractGradient(vn_autodiff);

  // Verify the result.
  EXPECT_TRUE(CompareMatrices(
      N, vn_derivs, kTolerance, MatrixCompareType::relative));

  // Automatically differentiate vt (with respect to v) to get the tangent
  // velocities Jacobian Jt.
  VectorX<AutoDiffXd> vt_autodiff = CalcTangentVelocities(
      *plant_autodiff, *context_autodiff, penetrations_, R_WC_set);
  const MatrixX<double> vt_derivs = math::ExtractGradient(vt_autodiff);

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

  MultibodyPlant<double> plant(0.0);
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
  DRAKE_EXPECT_NO_THROW(plant.GetJointByName(weld_name));
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

    // Only accelerations and joint reaction forces feedthrough, for either
    // continuous or discrete plants.
    EXPECT_TRUE(OnlyAccelerationAndReactionPortsFeedthrough(*plant_));

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

  // Ensure that call sites accepting a VectorBlock do not allocate.
  auto q_block = xc_expected.head(plant_->num_positions());
  auto v_block = xc_expected.tail(plant_->num_velocities());
  auto qv_block = xc_expected.head(plant_->num_multibody_states());
  {
    drake::test::LimitMalloc guard({.max_num_allocations = 0});
    plant_->SetPositions(context_.get(), q_block);
    plant_->SetVelocities(context_.get(), v_block);
    plant_->SetPositionsAndVelocities(context_.get(), qv_block);
  }
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
  EXPECT_EQ(plant_->num_multibody_states(), 28);

  EXPECT_EQ(plant_->num_positions(arm2), 7);
  EXPECT_EQ(plant_->num_velocities(arm2), 7);
  EXPECT_EQ(plant_->num_multibody_states(arm2), 14);

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

  // Ensure that call sites accepting a VectorBlock do not allocate.
  auto q_block = q.head(plant_->num_positions(arm2));
  auto v_block = qd.head(plant_->num_velocities(arm2));
  auto qv_block = x.head(q.size() + qd.size());
  {
    drake::test::LimitMalloc guard({.max_num_allocations = 0});
    plant_->SetPositions(context_.get(), arm2, q_block);
    plant_->SetPositions(*context_, &context_->get_mutable_state(),
                         arm2, q_block);
    plant_->SetVelocities(context_.get(), arm2, v_block);
    plant_->SetVelocities(*context_, &context_->get_mutable_state(),
                          arm2, v_block);
    plant_->SetPositionsAndVelocities(context_.get(), arm2, qv_block);
  }

  // Verify that we can retrieve the state vector using the output parameter
  // version and that populating these output vectors does not allocate any
  // heap.
  VectorX<double> q_out(q_block.size());
  VectorX<double> v_out(v_block.size());
  VectorX<double> qv_out(qv_block.size());
  {
    // Ensure that getters accepting an output vector do not allocate heap.
    drake::test::LimitMalloc guard({.max_num_allocations = 0});
    plant_->GetPositions(*context_, arm2, &q_out);
    plant_->GetVelocities(*context_, arm2, &v_out);
    plant_->GetPositionsAndVelocities(*context_, arm2, &qv_out);
  }
  // Verify values.
  EXPECT_EQ(q_out, q_block);
  EXPECT_EQ(v_out, v_block);
  EXPECT_EQ(qv_out, qv_block);

  // Verify error conditions.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->GetPositionsAndVelocities(*context_, arm2, &q_out),
      std::exception,
      "Output array is not properly sized.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->GetPositions(*context_, arm2, &qv_out),
      std::exception,
      "Output array is not properly sized.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->GetVelocities(*context_, arm2, &qv_out),
      std::exception,
      "Output array is not properly sized.");

  // Test the GetPositionsFromArray and GetVelocitiesFromArray functionality.
  // Use qv_out as the state vector.
  VectorX<double> q_out_array(q_block.size());
  VectorX<double> v_out_array(v_block.size());
  VectorX<double> state_vector = plant_->GetPositionsAndVelocities(*context_);

  {
    // Ensure that getters accepting an output vector do not allocate heap.
    drake::test::LimitMalloc guard({.max_num_allocations = 0});
    plant_->GetPositionsFromArray(arm2,
        state_vector.head(plant_->num_positions()), &q_out_array);
    plant_->GetVelocitiesFromArray(arm2,
        state_vector.tail(plant_->num_velocities()), &v_out_array);
  }

  // Verify values.
  EXPECT_EQ(q_out_array, q_block);
  EXPECT_EQ(v_out_array, v_block);

  // Verify GetPositionsFromArray and GetVelocitiesFromArray error case.
  VectorX<double> q_out_array_err(q_block.size()+1);
  VectorX<double> v_out_array_err(v_block.size()+1);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->GetPositionsFromArray(arm2,
        state_vector.head(plant_->num_positions()), &q_out_array_err),
      std::exception,
      "Output array is not properly sized.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->GetVelocitiesFromArray(arm2,
        state_vector.tail(plant_->num_velocities()), &v_out_array_err),
      std::exception,
      "Output array is not properly sized.");
}

// Verifies we instantiated an appropriate MultibodyPlant model based on the
// fixture's parameter.
TEST_P(KukaArmTest, CheckContinuousOrDiscreteModel) {
  // The plant must be a discrete system if the periodic update period is zero.
  EXPECT_EQ(!plant_->is_discrete(), this->GetParam() == 0);
}

INSTANTIATE_TEST_SUITE_P(
    Blank, KukaArmTest,
    testing::Values(0.0 /* continuous state */, 1e-3 /* discrete state */));

GTEST_TEST(StateSelection, JointHasNoActuator) {
  const std::string file_name =
      "drake/multibody/benchmarks/acrobot/acrobot.sdf";
  MultibodyPlant<double> plant(0.0);
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
  MultibodyPlant<double> plant(0.0);
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

  // Assert the base of the robot is free and modeled with a quaternion before
  // moving on with this assumption.
  ASSERT_TRUE(plant.GetBodyByName("iiwa_link_0").is_floating());
  ASSERT_TRUE(plant.GetBodyByName("iiwa_link_0").has_quaternion_dofs());

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

  const auto& left_finger_sliding_joint =
      plant.GetJointByName("left_finger_sliding_joint");
  const auto& right_finger_sliding_joint =
      plant.GetJointByName("right_finger_sliding_joint");

  // We build a state selector for the gripper dofs.
  std::vector<JointIndex> gripper_selected_joints;
  gripper_selected_joints.push_back(
      left_finger_sliding_joint.index());  // user index = 0.
  gripper_selected_joints.push_back(
      right_finger_sliding_joint.index());  // user index = 1.

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
  Sx_gripper_expected(0, left_finger_sliding_joint.position_start()) = 1;
  // second joint position, right finger.
  Sx_gripper_expected(1, right_finger_sliding_joint.position_start()) = 1;
  // first joint velocity, left finger.
  Sx_gripper_expected(2, plant.num_positions() +
                             left_finger_sliding_joint.velocity_start()) = 1;
  // second joint velocity, right finger.
  Sx_gripper_expected(3, plant.num_positions() +
                             right_finger_sliding_joint.velocity_start()) = 1;
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

  // Fill in the block for the IIWA's actuators.
  auto B_iiwa = B_expected.block(6 /* skip floating base */, 0,
                                 7 /* iiwa joints */, 7 /* iiwa actuators */);
  B_iiwa.setIdentity();

  // Fill in the block for the gripper's actuators.
  const auto& left_finger_actuator =
      plant.GetJointActuatorByName("left_finger_sliding_joint");
  const auto& right_finger_actuator =
      plant.GetJointActuatorByName("right_finger_sliding_joint");
  B_expected(left_finger_actuator.joint().velocity_start(),
             int{left_finger_actuator.index()}) = 1;
  B_expected(right_finger_actuator.joint().velocity_start(),
             int{right_finger_actuator.index()}) = 1;
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

  MultibodyPlant<double> plant(0.0);

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
  const RigidTransformd X_WLink0(Vector3d(0, 0, table_top_z_in_world));
  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("iiwa_link_0", arm_model),
      X_WLink0);

  // Load a second table for objects.
  const ModelInstanceIndex objects_table_model =
      parser.AddModelFromFile(table_sdf_path, "objects_table");
  const RigidTransformd X_WT(Vector3d(0.8, 0.0, 0.0));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", objects_table_model), X_WT);

  // Define a fixed frame on the -x, -y corner of the objects table.
  const RigidTransformd X_TO(RotationMatrixd::MakeXRotation(-M_PI_2),
                             Vector3d(-0.3, -0.3, table_top_z_in_world));
  const auto& objects_frame_O =
      plant.AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          "objects_frame", plant.GetFrameByName("link", objects_table_model),
          X_TO));

  // Add a floating mug.
  const ModelInstanceIndex mug_model =
      parser.AddModelFromFile(mug_sdf_path);
  const Body<double>& mug = plant.GetBodyByName("main_body", mug_model);

  plant.Finalize();

  // Assert that the mug is a free body before moving on with this assumption.
  ASSERT_TRUE(mug.is_floating());
  ASSERT_TRUE(mug.has_quaternion_dofs());
  EXPECT_EQ(mug.floating_position_suffix(0), "qw");
  EXPECT_EQ(mug.floating_position_suffix(1), "qx");
  EXPECT_EQ(mug.floating_position_suffix(2), "qy");
  EXPECT_EQ(mug.floating_position_suffix(3), "qz");
  EXPECT_EQ(mug.floating_position_suffix(4), "x");
  EXPECT_EQ(mug.floating_position_suffix(5), "y");
  EXPECT_EQ(mug.floating_position_suffix(6), "z");
  EXPECT_EQ(mug.floating_velocity_suffix(0), "wx");
  EXPECT_EQ(mug.floating_velocity_suffix(1), "wy");
  EXPECT_EQ(mug.floating_velocity_suffix(2), "wz");
  EXPECT_EQ(mug.floating_velocity_suffix(3), "vx");
  EXPECT_EQ(mug.floating_velocity_suffix(4), "vy");
  EXPECT_EQ(mug.floating_velocity_suffix(5), "vz");


  // The "world" is not considered as a free body.
  EXPECT_FALSE(plant.world_body().is_floating());

  // Sanity check that bodies welded to the world are not free.
  EXPECT_FALSE(plant.GetBodyByName("iiwa_link_0").is_floating());
  EXPECT_FALSE(plant.GetBodyByName("link", objects_table_model).is_floating());

  std::unordered_set<BodyIndex> expected_floating_bodies({mug.index()});
  auto floating_bodies = plant.GetFloatingBaseBodies();
  EXPECT_EQ(expected_floating_bodies, floating_bodies);

  // Check link 0 is anchored, and link 1 is not.
  EXPECT_TRUE(plant.IsAnchored(plant.GetBodyByName("iiwa_link_0", arm_model)));
  EXPECT_FALSE(
      plant.IsAnchored(plant.GetBodyByName("iiwa_link_1", arm_model)));

  auto context = plant.CreateDefaultContext();

  // Initialize the pose X_OM of the mug frame M in the objects table frame O.
  const Vector3d p_OoMo_O(0.05, 0.0, 0.05);
  const RigidTransformd X_OM(p_OoMo_O);
  plant.SetFreeBodyPoseInAnchoredFrame(
      context.get(), objects_frame_O, mug, X_OM);

  // Retrieve the pose of the mug in the world.
  const RigidTransformd& X_WM = plant.EvalBodyPoseInWorld(*context, mug);

  const RigidTransformd X_WM_expected = X_WT * X_TO * X_OM;

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_WM.GetAsMatrix4(), X_WM_expected.GetAsMatrix4(),
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
  MultibodyPlant<double> plant(0.0);
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
  const math::RotationMatrix<double> X_WB_new(RollPitchYawd(0.3, 0.4, 0.5));
  plant.SetFreeBodyRandomRotationDistribution(
      body, X_WB_new.cast<symbolic::Expression>().ToQuaternion());

  plant.SetRandomContext(context.get(), &generator);
  X_WB = plant.GetFreeBodyPose(*context, body);

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(
      X_WB_new.matrix(), X_WB.rotation().matrix(),
      kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(MultibodyPlantTest, SceneGraphPorts) {
    MultibodyPlant<double> plant(0.0);

    MultibodyPlant<double> plant_finalized(0.0);
    plant_finalized.Finalize();

    // Test that SceneGraph ports exist and are accessible, both pre and post
    // finalize, without the presence of a connected SceneGraph.
    EXPECT_NO_THROW(plant.get_geometry_query_input_port());
    EXPECT_NO_THROW(plant.get_geometry_poses_output_port());
    EXPECT_NO_THROW(plant_finalized.get_geometry_query_input_port());
    EXPECT_NO_THROW(plant_finalized.get_geometry_poses_output_port());
}

GTEST_TEST(MultibodyPlantTest, RigidBodyParameters) {
  // Add a plant with a few rigid bodies.
  MultibodyPlant<double> plant(0.0);

  const double sphere_radius = 1.0;
  const double sphere_mass = 2.5;
  const Vector3d sphere_com(0, 0, 0);
  const UnitInertia<double> sphere_unit_inertia =
      UnitInertia<double>::SolidSphere(sphere_radius);
  const RigidBody<double>& sphere = plant.AddRigidBody(
      "sphere",
      SpatialInertia<double>(sphere_mass, sphere_com, sphere_unit_inertia));

  const double cube_length = 2.0;
  const double cube_mass = 5.0;
  const Vector3d cube_com(0, 0, 0);
  const UnitInertia<double> cube_unit_inertia =
      UnitInertia<double>::SolidBox(cube_length, cube_length, cube_length);
  const RigidBody<double>& cube = plant.AddRigidBody(
      "cube", SpatialInertia<double>(cube_mass, cube_com, cube_unit_inertia));

  plant.Finalize();

  // Create a default context.
  auto context = plant.CreateDefaultContext();

  // Verify default parameters exist and are correct.
  const double sphere_mass_in_context = sphere.get_mass(*context);
  const Vector3<double> sphere_com_in_context =
      sphere.CalcCenterOfMassInBodyFrame(*context);
  const SpatialInertia<double> sphere_inertia_in_context =
      sphere.CalcSpatialInertiaInBodyFrame(*context);

  const double cube_mass_in_context = cube.get_mass(*context);
  const Vector3<double> cube_com_in_context =
      cube.CalcCenterOfMassInBodyFrame(*context);
  const SpatialInertia<double> cube_inertia_in_context =
      cube.CalcSpatialInertiaInBodyFrame(*context);

  EXPECT_EQ(sphere_mass_in_context, sphere_mass);
  EXPECT_EQ(sphere_inertia_in_context.get_mass(), sphere_mass);
  EXPECT_TRUE(CompareMatrices(sphere_com_in_context, sphere_com));
  EXPECT_TRUE(CompareMatrices(sphere_inertia_in_context.get_com(), sphere_com));
  EXPECT_TRUE(CompareMatrices(
      sphere_inertia_in_context.get_unit_inertia().get_moments(),
      sphere_unit_inertia.get_moments()));
  EXPECT_TRUE(CompareMatrices(
      sphere_inertia_in_context.get_unit_inertia().get_products(),
      sphere_unit_inertia.get_products()));

  EXPECT_EQ(cube_mass_in_context, cube_mass);
  EXPECT_EQ(cube_inertia_in_context.get_mass(), cube_mass);
  EXPECT_TRUE(CompareMatrices(cube_com_in_context, cube_com));
  EXPECT_TRUE(CompareMatrices(cube_inertia_in_context.get_com(), cube_com));
  EXPECT_TRUE(
      CompareMatrices(cube_inertia_in_context.get_unit_inertia().get_moments(),
                      cube_unit_inertia.get_moments()));
  EXPECT_TRUE(
      CompareMatrices(cube_inertia_in_context.get_unit_inertia().get_products(),
                      cube_unit_inertia.get_products()));

  // Change parameters.
  const double new_sphere_radius = 1.5;
  const double new_sphere_mass = 3.9;
  const Vector3d new_sphere_com(0, 0, 0);
  const UnitInertia<double> new_sphere_unit_inertia =
      UnitInertia<double>::SolidSphere(new_sphere_radius);

  const double new_cube_length = 1.3;
  const double new_cube_mass = 3.0;
  const Vector3d new_cube_com(0, 0, 0);
  const UnitInertia<double> new_cube_unit_inertia =
      UnitInertia<double>::SolidBox(new_cube_length, new_cube_length,
                                    new_cube_length);

  SpatialInertia<double> new_sphere_params(new_sphere_mass, new_sphere_com,
                                           new_sphere_unit_inertia);

  SpatialInertia<double> new_cube_params(new_cube_mass, new_cube_com,
                                         new_cube_unit_inertia);

  sphere.SetSpatialInertiaInBodyFrame(context.get(), new_sphere_params);
  cube.SetSpatialInertiaInBodyFrame(context.get(), new_cube_params);

  // Verify parameters propagate.
  const double new_sphere_mass_in_context = sphere.get_mass(*context);
  const Vector3<double> new_sphere_com_in_context =
      sphere.CalcCenterOfMassInBodyFrame(*context);
  const SpatialInertia<double> new_sphere_inertia_in_context =
      sphere.CalcSpatialInertiaInBodyFrame(*context);

  const double new_cube_mass_in_context = cube.get_mass(*context);
  const Vector3<double> new_cube_com_in_context =
      cube.CalcCenterOfMassInBodyFrame(*context);
  const SpatialInertia<double> new_cube_inertia_in_context =
      cube.CalcSpatialInertiaInBodyFrame(*context);

  EXPECT_EQ(new_sphere_mass_in_context, new_sphere_mass);
  EXPECT_EQ(new_sphere_inertia_in_context.get_mass(), new_sphere_mass);
  EXPECT_TRUE(
      CompareMatrices(new_sphere_com_in_context, new_sphere_com));
  EXPECT_TRUE(
      CompareMatrices(new_sphere_inertia_in_context.get_com(), new_sphere_com));
  EXPECT_TRUE(CompareMatrices(
      new_sphere_inertia_in_context.get_unit_inertia().get_moments(),
      new_sphere_unit_inertia.get_moments()));
  EXPECT_TRUE(CompareMatrices(
      new_sphere_inertia_in_context.get_unit_inertia().get_products(),
      new_sphere_unit_inertia.get_products()));

  EXPECT_EQ(new_cube_mass_in_context, new_cube_mass);
  EXPECT_EQ(new_cube_inertia_in_context.get_mass(), new_cube_mass);
  EXPECT_TRUE(
      CompareMatrices(new_cube_com_in_context, new_cube_com));
  EXPECT_TRUE(
      CompareMatrices(new_cube_inertia_in_context.get_com(), new_cube_com));
  EXPECT_TRUE(CompareMatrices(
      new_cube_inertia_in_context.get_unit_inertia().get_moments(),
      new_cube_unit_inertia.get_moments()));
  EXPECT_TRUE(CompareMatrices(
      new_cube_inertia_in_context.get_unit_inertia().get_products(),
      new_cube_unit_inertia.get_products()));
}

GTEST_TEST(MultibodyPlantTest, AutoDiffAcrobotParameters) {
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

  // Create an Acrobot plant with autodiff parameters for length and mass.
  const double m1 = 2.0;
  const double m2 = 4.0;
  const double l1 = 1.0;
  const double l2 = 2.0;
  const double lc1 = 0.5 * l1;
  const double lc2 = 0.5 * l2;
  const double Gc1 = (1.0 / 12.0) * l1 * l1;
  const double Gc2 = (1.0 / 12.0) * l2 * l2;
  const double Ic1 = m1 * Gc1;
  const double Ic2 = m2 * Gc2;

  const AcrobotParameters params(m1, m2, l1, l2, lc1, lc2, Ic1, Ic2, 0.0, 0.0,
                                 9.81);
  unique_ptr<MultibodyPlant<double>> plant = MakeAcrobotPlant(params, true);

  // Create a default context and set state.
  unique_ptr<Context<double>> context = plant->CreateDefaultContext();

  const RevoluteJoint<double>& shoulder_joint =
      plant->GetJointByName<RevoluteJoint>(params.shoulder_joint_name());
  const RevoluteJoint<double>& elbow_joint =
      plant->GetJointByName<RevoluteJoint>(params.elbow_joint_name());
  shoulder_joint.set_angle(context.get(), 0.0);
  elbow_joint.set_angle(context.get(), 0.0);

  // Scalar convert the plant to AutoDiffXd.
  unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(*plant);
  unique_ptr<Context<AutoDiffXd>> context_autodiff =
      plant_autodiff->CreateDefaultContext();
  context_autodiff->SetTimeStateAndParametersFrom(*context);

  // Set up our parameters as independent variables.
  // We will use mass and length of the two links of the acrobot as our
  // variables to differentiate with respect to. We must update multibody
  // components that depend on these parameters so that their derivatives will
  // propagate. For this example the multibody elements that require parameter
  // updates are the bodies' mass and inertia, and the joints' offset frames.
  const AutoDiffXd m1_ad(m1, Vector4<double>(1, 0, 0, 0));
  const AutoDiffXd m2_ad(m2, Vector4<double>(0, 1, 0, 0));
  const AutoDiffXd l1_ad(l1, Vector4<double>(0, 0, 1, 0));
  const AutoDiffXd l2_ad(l2, Vector4<double>(0, 0, 0, 1));
  const AutoDiffXd lc1_ad = 0.5 * l1_ad;
  const AutoDiffXd lc2_ad = 0.5 * l2_ad;
  const AutoDiffXd Gc1_ad = (1.0 / 12.0) * l1_ad * l1_ad;
  const AutoDiffXd Gc2_ad = (1.0 / 12.0) * l2_ad * l2_ad;

  // Differentiable parameters for the acrobot's mass/inertia parameters.
  // Frame L1's origin is located at the shoulder outboard frame.
  const Vector3<AutoDiffXd> p_L1L1cm = -lc1_ad * Vector3d::UnitZ();
  // Frame L2's origin is located at the elbow outboard frame.
  const Vector3<AutoDiffXd> p_L2L2cm = -lc2_ad * Vector3d::UnitZ();

  // Define each link's spatial inertia about their respective COM.
  UnitInertia<AutoDiffXd> Gc1_Bcm =
      UnitInertia<AutoDiffXd>::StraightLine(Gc1_ad, Vector3d::UnitZ());
  SpatialInertia<AutoDiffXd> M1_L1o =
      SpatialInertia<AutoDiffXd>::MakeFromCentralInertia(m1_ad, p_L1L1cm,
                                                         Gc1_Bcm * m1_ad);

  UnitInertia<AutoDiffXd> Gc2_Bcm =
      UnitInertia<AutoDiffXd>::StraightLine(Gc2_ad, Vector3d::UnitZ());
  SpatialInertia<AutoDiffXd> M2_L2o =
      SpatialInertia<AutoDiffXd>::MakeFromCentralInertia(m2_ad, p_L2L2cm,
                                                         Gc2_Bcm * m2_ad);

  // Update each body's inertial parameters.
  plant_autodiff->GetRigidBodyByName(params.link1_name())
      .SetSpatialInertiaInBodyFrame(context_autodiff.get(), M1_L1o);

  plant_autodiff->GetRigidBodyByName(params.link2_name())
      .SetSpatialInertiaInBodyFrame(context_autodiff.get(), M2_L2o);

  // The parent frame of the elbow joint is an offset frame that depends on the
  // length of link1. Therefore, we must update this offset frame's parameter
  // with the autodiff variable containing the proper partial derivatives for
  // them to propagate through dynamics computations.
  const RevoluteJoint<AutoDiffXd>& elbow_joint_ad =
      plant_autodiff->GetJointByName<RevoluteJoint>(params.elbow_joint_name());
  const FixedOffsetFrame<AutoDiffXd>& elbow_joint_parent_frame =
      dynamic_cast<const FixedOffsetFrame<AutoDiffXd>&>(
          elbow_joint_ad.frame_on_parent());

  const RigidTransform<AutoDiffXd> X_link1_Ei(-l1_ad * Vector3d::UnitZ());

  elbow_joint_parent_frame.SetPoseInBodyFrame(context_autodiff.get(),
                                              X_link1_Ei);

  // Take the derivative of the mass matrix w.r.t. length.
  Matrix2<AutoDiffXd> mass_matrix;
  plant_autodiff->CalcMassMatrix(*context_autodiff, &mass_matrix);

  const auto& mass_matrix_grad = math::ExtractGradient(mass_matrix);

  // Verify numerical derivative matches analytic solution.
  // In the starting configuration q = (0, 0).
  //
  //   c1 = cos(q[0]) = 1
  //   s1 = sin(q[0]) = 0
  //   c2 = cos(q[1]) = 1
  //   s2 = sin(q[1]) = 0
  //
  //  Moments of Inertia, taken about the pivots:
  //
  //    I₁ = 4m₁Ic₁ = (1/3)m₁l₁²
  //    I₂ = 4m₂Ic₂ = (1/3)m₂l₂²
  //
  // Moment of inertia at the shoulder joint origin.
  const double I1 = 4 * Ic1;
  // Moment of inertia at the elbow joint origin.
  const double I2 = 4 * Ic2;

  // Analytic Mass Matrix, M:
  //
  //  [ I₁ + I₂ + m₂l₁² + 2m₂l₁lc₂c₂   I₂ + m₂l₁lc₂c₂ ]
  //  [      I₂ + m₂l₁lc₂c₂                 I₂        ]
  Matrix2<double> analytic_mass_matrix;
  analytic_mass_matrix << I1 + I2 + m2*l1*l1 + 2*m2*l1*lc2, I2 + m2*l1*lc2,
                                            I2 + m2*l1*lc2,             I2;
  EXPECT_TRUE(CompareMatrices(mass_matrix, analytic_mass_matrix, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂m₁:
  // [ (1/3)l₁²      0 ]
  // [     0         0 ]
  Vector4<double> analytic_mass_matrix_partial_m1;
  analytic_mass_matrix_partial_m1 << (1.0/3.0)*l1*l1, 0.0,
                                                 0.0, 0.0;
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(0),
                              analytic_mass_matrix_partial_m1, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂m₂:
  // [ (1/3)l₂² + l₁² + 2l₁lc₂c₂     (1/3)l₂² + l₁lc₂c₂ ]
  // [    (1/3)l₂² + l₁lc₂c₂               (1/3)l₂²     ]
  Vector4<double> analytic_mass_matrix_partial_m2;
  analytic_mass_matrix_partial_m2 <<
      (1.0/3.0)*l2*l2 + l1*l1 + 2*l1*lc2, (1.0/3.0)*l2*l2 + l1*lc2,
                (1.0/3.0)*l2*l2 + l1*lc2,          (1.0/3.0)*l2*l2;
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(1),
                              analytic_mass_matrix_partial_m2, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂l₁:
  //  [ (2/3)m₁l₁ + 2m₂l₁ + 2m₂lc₂c₂   m₂lc₂c₂ ]
  //  [        m₂lc₂c₂                    0    ]
  Vector4<double> analytic_mass_matrix_partial_l1;
  analytic_mass_matrix_partial_l1 <<
     (2.0/3.0)*m1*l1 + 2*m2*l1 + 2*m2*lc2, m2*lc2,
                                   m2*lc2,      0;
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(2),
                              analytic_mass_matrix_partial_l1, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂l₂:
  // [   (2/3)m₂l₂ + m₂l₁       (2/3)m₂l₂ + (1/2)m₂l₁ ]
  // [ (2/3)m₂l₂ + (1/2)m₂l₁          (2/3)m₂l₂       ]
  Vector4<double> analytic_mass_matrix_partial_l2;
  analytic_mass_matrix_partial_l2 <<
          (2.0/3.0)*m2*l2 + m2*l1, (2.0/3.0)*m2*l2 + 0.5*m2*l1,
      (2.0/3.0)*m2*l2 + 0.5*m2*l1,             (2.0/3.0)*m2*l2;
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(3),
                              analytic_mass_matrix_partial_l2, kTolerance,
                              MatrixCompareType::relative));
}

GTEST_TEST(MultibodyPlantTests, FixedOffsetFrameParameters) {
  MultibodyPlant<double> plant(0.0);

  const Vector3d p_WF(0, 0, 0);
  const RotationMatrixd R_WF{};
  const RigidTransformd X_WF(R_WF, p_WF);

  const Body<double>& body = plant.AddRigidBody("B", SpatialInertia<double>{});;

  const Joint<double>& weld_joint =
      plant.AddJoint<WeldJoint>("weld_WB", plant.world_body(), X_WF, body, {},
                                RigidTransformd::Identity());

  const FixedOffsetFrame<double>& frame_F =
      dynamic_cast<const FixedOffsetFrame<double>&>(
          weld_joint.frame_on_parent());

  plant.Finalize();

  // Create a default context.
  auto context = plant.CreateDefaultContext();

  // Verify default parameters are set for the frame and that they've propagated
  // to the welded body's pose.
  const RigidTransformd& X_WF_context = frame_F.CalcPoseInBodyFrame(*context);
  const math::RigidTransformd& X_WF_body =
      plant.EvalBodyPoseInWorld(*context, body);

  EXPECT_TRUE(
      CompareMatrices(X_WF.GetAsMatrix34(), X_WF_context.GetAsMatrix34()));
  EXPECT_TRUE(CompareMatrices(X_WF.GetAsMatrix34(), X_WF_body.GetAsMatrix34()));

  // Set new parameters and verify they propagate.
  const Vector3d p_WF_new(1, 1, 1);
  const RotationMatrixd R_WF_new = RotationMatrixd::MakeXRotation(3);
  const RigidTransformd X_WF_new(R_WF_new, p_WF_new);

  frame_F.SetPoseInBodyFrame(context.get(), X_WF_new);

  const RigidTransformd& X_WF_context_new =
      frame_F.CalcPoseInBodyFrame(*context);
  const math::RigidTransformd& X_WF_body_new =
      plant.EvalBodyPoseInWorld(*context, body);
  EXPECT_TRUE(CompareMatrices(X_WF_new.GetAsMatrix34(),
                              X_WF_context_new.GetAsMatrix34()));
  EXPECT_TRUE(
      CompareMatrices(X_WF_new.GetAsMatrix34(), X_WF_body_new.GetAsMatrix34()));
}

GTEST_TEST(MultibodyPlant, CombinePointContactParameters) {
  // case: k1+k2 == 0.0.
  {
    const auto [k, d] = internal::CombinePointContactParameters(0., 0., 0., 0.);
    EXPECT_TRUE(k == 0 && d == 0);
  }
  // case: k1+k2 != 0.0.
  {
    const auto [k, d] = internal::CombinePointContactParameters(1., 1., 1., 1.);
    double kEps = std::numeric_limits<double>::epsilon();
    EXPECT_NEAR(k, 0.5, 4 * kEps);
    EXPECT_NEAR(d, 1.0, 4 * kEps);
  }
}

// Demonstrate that FixInputPortsFrom does *not* currently work for a
// MultibodyPlant if the other MultibodyPlant was connected to a
// SceneGraph. This is because the geometry query input port is a QueryValue<T>,
// and FixInputPortsFrom does not convert its scalar type.
// TODO(5454) Once transmogrification of scalar-dependent abstract values is
// implemented, this test can simply be removed (as we no longer have to track
// this undesirable behavior).
GTEST_TEST(MultibodyPlant, FixInputPortsFrom) {
  systems::DiagramBuilder<double> builder;
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant).AddModelFromFile(
      FindResourceOrThrow("drake/multibody/plant/test/split_pendulum.sdf"));
  plant.Finalize();
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();
  auto& plant_context = plant.GetMyContextFromRoot(*context);

  // Convert only the plant to autodiff.
  auto autodiff_plant = plant.ToAutoDiffXd();
  auto autodiff_context = autodiff_plant->CreateDefaultContext();

  DRAKE_EXPECT_THROWS_MESSAGE(autodiff_plant->FixInputPortsFrom(
                                  plant, plant_context, autodiff_context.get()),
                              ".*FixInputPortTypeCheck.*");
}

}  // namespace
}  // namespace multibody
}  // namespace drake
