#include "drake/multibody/plant/multibody_plant.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <numbers>
#include <regex>
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
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
#include "drake/multibody/plant/test_utilities/multibody_plant_remodeling.h"
#include "drake/multibody/test_utilities/add_fixed_objects_to_plant.h"
#include "drake/multibody/tree/door_hinge.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/revolute_spring.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/universal_joint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {

using Eigen::AngleAxisd;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Translation3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::FrameId;
using geometry::FramePoseVector;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::IllustrationProperties;
using geometry::PenetrationAsPointPair;
using geometry::PerceptionProperties;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::internal::DummyRenderEngine;
using geometry::render::RenderLabel;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrix;
using math::RotationMatrixd;
using multibody::MultibodyForces;
using multibody::Parser;
using multibody::benchmarks::Acrobot;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::benchmarks::pendulum::MakePendulumPlant;
using multibody::benchmarks::pendulum::PendulumParameters;
using std::make_pair;
using std::pair;
using std::tie;
using std::unique_ptr;
using systems::BasicVector;
using systems::ConstantVectorSource;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Linearize;
using systems::LinearSystem;
using systems::OutputPortIndex;
using systems::VectorBase;

namespace multibody {

class MultibodyPlantTester {
 public:
  MultibodyPlantTester() = delete;

  template <typename T>
  static BodyIndex FindBodyByGeometryId(const MultibodyPlant<T>& plant,
                                        GeometryId id) {
    return plant.FindBodyByGeometryId(id);
  }

  template <typename T>
  static void AddJointActuationForces(const MultibodyPlant<T>& plant,
                                      const systems::Context<T>& context,
                                      VectorX<T>* forces) {
    plant.AddJointActuationForces(context, forces);
  }
};

namespace {

// Verifies that fresh-constructed plants are using the default contact surface
// representation.
GTEST_TEST(MultibodyPlant, GetDefaultContactSurfaceRepresentation) {
  for (const double time_step : {0.0, 0.1}) {
    MultibodyPlant<double> plant{time_step};
    EXPECT_EQ(plant.get_contact_surface_representation(),
              MultibodyPlant<double>::GetDefaultContactSurfaceRepresentation(
                  time_step));
  }
}

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
      Parser(plant.get())
          .AddModelsFromUrl(
              "package://drake/multibody/plant/test/split_pendulum.sdf")
          .at(0);
  EXPECT_EQ(plant->num_model_instances(), 3);

  plant->Finalize();
  // We should throw an exception if finalize is called twice.  Verify this.
  EXPECT_THROW(plant->Finalize(), std::logic_error);

  // Verify that a non-positive penetration_allowance throws an exception.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->set_penetration_allowance(-1),
      "set_penetration_allowance\\(\\): penetration_allowance must be strictly "
      "positive.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->set_penetration_allowance(0),
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

  // Acrobot is in the default model instance.
  EXPECT_EQ(plant->num_actuators(default_model_instance()), 1);
  EXPECT_EQ(plant->num_actuated_dofs(default_model_instance()), 1);
  EXPECT_EQ(plant->num_positions(default_model_instance()), 2);
  EXPECT_EQ(plant->num_velocities(default_model_instance()), 2);

  // Pendulum model instance.
  EXPECT_EQ(plant->num_actuators(pendulum_model_instance), 1);
  EXPECT_EQ(plant->num_actuated_dofs(pendulum_model_instance), 1);
  EXPECT_EQ(plant->num_positions(pendulum_model_instance), 1);
  EXPECT_EQ(plant->num_velocities(pendulum_model_instance), 1);

  // Check that the input/output ports have the appropriate geometry.
  EXPECT_EQ(plant->get_actuation_input_port(default_model_instance()).size(),
            1);
  EXPECT_EQ(plant->get_actuation_input_port(pendulum_model_instance).size(), 1);
  EXPECT_EQ(plant->get_actuation_input_port().size(), 2);
  EXPECT_EQ(plant->get_state_output_port().size(), 6);
  EXPECT_EQ(plant->get_state_output_port(default_model_instance()).size(), 4);
  EXPECT_EQ(plant->get_state_output_port(pendulum_model_instance).size(), 2);

  // Check that model-instance ports get named properly.
  EXPECT_TRUE(plant->HasModelInstanceNamed("DefaultModelInstance"));
  EXPECT_TRUE(plant->HasModelInstanceNamed("SplitPendulum"));
  EXPECT_EQ(
      plant->get_actuation_input_port(default_model_instance()).get_name(),
      "DefaultModelInstance_actuation");
  EXPECT_EQ(plant->get_state_output_port(default_model_instance()).get_name(),
            "DefaultModelInstance_state");
  EXPECT_EQ(plant->get_state_output_port(pendulum_model_instance).get_name(),
            "SplitPendulum_state");

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
  const RigidBody<double>& link1 =
      plant->GetBodyByName(parameters.link1_name());
  EXPECT_EQ(link1.name(), parameters.link1_name());
  EXPECT_EQ(link1.model_instance(), default_model_instance());

  const RigidBody<double>& link2 =
      plant->GetBodyByName(parameters.link2_name());
  EXPECT_EQ(link2.name(), parameters.link2_name());
  EXPECT_EQ(link2.model_instance(), default_model_instance());

  const RigidBody<double>& upper = plant->GetBodyByName("upper_section");
  EXPECT_EQ(upper.model_instance(), pendulum_model_instance);

  const RigidBody<double>& lower = plant->GetBodyByName("lower_section");
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
  const Joint<double>& pin_joint = plant->GetJointByName("pin");
  EXPECT_EQ(pin_joint.model_instance(), pendulum_model_instance);
  const Joint<double>& weld_joint = plant->GetJointByName("weld");
  EXPECT_EQ(weld_joint.model_instance(), pendulum_model_instance);
  EXPECT_THROW(plant->GetJointByName(kInvalidName), std::logic_error);

  // Get force elements by index. In this case the default gravity field.
  const ForceElementIndex gravity_field_index(0);
  EXPECT_EQ(
      &plant->gravity_field(),
      &plant->GetForceElement<UniformGravityFieldElement>(gravity_field_index));
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->GetForceElement<RevoluteSpring>(gravity_field_index),
      ".*not of type .*RevoluteSpring.* but of type "
      ".*UniformGravityFieldElement.*");
  const ForceElementIndex invalid_force_index(plant->num_force_elements() + 1);
  EXPECT_ANY_THROW(plant->GetForceElement<RevoluteSpring>(invalid_force_index));

  // Get joint indices by model instance.
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

  // Get actuated joint indices only, by model instance.
  const std::vector<JointIndex> acrobot_actuated_joint_indices =
      plant->GetActuatedJointIndices(default_model_instance());
  EXPECT_EQ(acrobot_actuated_joint_indices.size(), 1);
  EXPECT_EQ(acrobot_actuated_joint_indices[0], elbow_joint.index());

  const std::vector<JointIndex> pendulum_actuated_joint_indices =
      plant->GetActuatedJointIndices(pendulum_model_instance);
  EXPECT_EQ(pendulum_actuated_joint_indices.size(), 1);  // pin joint
  EXPECT_EQ(pendulum_actuated_joint_indices[0], pin_joint.index());

  // Get actuated joint actuator indices by model instance.
  const std::vector<JointActuatorIndex> acrobot_joint_actuator_indices =
      plant->GetJointActuatorIndices(default_model_instance());
  EXPECT_EQ(acrobot_joint_actuator_indices.size(), 1);
  EXPECT_EQ(plant->get_joint_actuator(acrobot_joint_actuator_indices[0])
                .joint()
                .index(),
            elbow_joint.index());

  const std::vector<JointActuatorIndex> pendulum_joint_actuator_indices =
      plant->GetJointActuatorIndices(pendulum_model_instance);
  EXPECT_EQ(pendulum_joint_actuator_indices.size(), 1);  // pin joint
  EXPECT_EQ(plant->get_joint_actuator(pendulum_joint_actuator_indices[0])
                .joint()
                .index(),
            pin_joint.index());

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
      ".*not of type .*PrismaticJoint.* but of type "
      ".*RevoluteJoint.*");

  // MakeAcrobotPlant() has already called Finalize() on the acrobot model.
  // Therefore no more modeling elements can be added. Verify this.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->AddRigidBody("AnotherBody", default_model_instance(),
                          SpatialInertia<double>::NaN()),
      "Post-finalize calls to '.*' are not allowed; "
      "calls to this method must happen before Finalize\\(\\).");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->AddJoint<RevoluteJoint>("AnotherJoint", link1, std::nullopt, link2,
                                     std::nullopt, Vector3d::UnitZ()),
      "Post-finalize calls to '.*' are not allowed; "
      "calls to this method must happen before Finalize\\(\\).");
  // Test API for simplified `AddJoint` method.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->AddJoint(std::make_unique<RevoluteJoint<double>>(
          "AnotherJoint", link1.body_frame(), link2.body_frame(),
          Vector3d::UnitZ())),
      "Post-finalize calls to '.*' are not allowed; "
      "calls to this method must happen before Finalize\\(\\).");
  // TODO(amcastro-tri): add test to verify that requesting a joint of the wrong
  //  type throws an exception. We need another joint type to do so.

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

GTEST_TEST(MultibodyPlantTest, AddJointActuator) {
  MultibodyPlant<double> plant(0.0);
  ModelInstanceIndex model_instance = plant.AddModelInstance("instance");
  const auto M_B = SpatialInertia<double>::NaN();
  auto body = &plant.AddRigidBody("body", model_instance, M_B);
  const Joint<double>& planar_joint =
      plant.AddJoint(std::make_unique<PlanarJoint<double>>(
          "planar", plant.world_body().body_frame(), body->body_frame(),
          Eigen::Vector3d{0, 0, 0.1}));

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddJointActuator("planar_actuator", planar_joint),
      ".*3 degrees of freedom.*");
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

  // Check support for C++ structured binding.
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
  const char kSdfUrl[] =
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
  auto plant =
      std::make_unique<MultibodyPlant<double>>(0 /* plant type irrelevant */);
  Parser parser(plant.get());
  multibody::ModelInstanceIndex iiwa_instance =
      parser.AddModelsFromUrl(kSdfUrl).at(0);
  plant->Finalize();

  // Use string to ensure that there is no heap allocation in the implicit
  // conversion to string_view.
  const std::string kLinkName = "iiwa_link_0";
  const std::string kJointName = "iiwa_joint_1";

  // Note that failed queries cause exceptions to be thrown (which allocates
  // heap).
  drake::test::LimitMalloc dummy;

  // Check the HasX versions first. Note that functions that take no model
  // instance argument delegate to model instance argument versions.
  EXPECT_TRUE(plant->HasModelInstanceNamed("iiwa14"));
  EXPECT_TRUE(plant->HasBodyNamed(kLinkName, iiwa_instance));
  EXPECT_TRUE(plant->HasFrameNamed(kLinkName, iiwa_instance));
  EXPECT_TRUE(plant->HasJointNamed(kJointName, iiwa_instance));
  EXPECT_TRUE(plant->HasJointActuatorNamed(kJointName, iiwa_instance));

  // Check the GetX versions now.
  plant->GetModelInstanceByName("iiwa14");
  plant->GetBodyByName(kLinkName, iiwa_instance);
  plant->GetFrameByName(kLinkName, iiwa_instance);
  plant->GetJointByName(kJointName, iiwa_instance);
  plant->GetJointActuatorByName(kJointName, iiwa_instance);
}

// This test creates an empty model and checks simple invariants on model
// elements. We test the contracts here, not in MultibodyTree, to test
// behavior from public API perspective.
GTEST_TEST(MultibodyPlant, EmptyWorldElements) {
  const double time_step = 0.0;
  MultibodyPlant<double> plant(time_step);
  // Model instances.
  EXPECT_EQ(plant.num_model_instances(), 2);
  // Bodies.
  EXPECT_EQ(plant.num_bodies(), 1);
  const RigidBody<double>& world_body = plant.world_body();
  EXPECT_EQ(world_body.index(), world_index());
  EXPECT_EQ(world_body.model_instance(), world_model_instance());
  // Frames.
  EXPECT_EQ(plant.num_frames(), 1);
  const Frame<double>& world_frame = plant.world_frame();
  EXPECT_EQ(world_frame.index(), world_frame_index());
  EXPECT_EQ(world_frame.model_instance(), world_model_instance());
  EXPECT_EQ(&world_body.body_frame(), &world_frame);
  // Remaining elements.
  EXPECT_EQ(plant.num_joints(), 0);
  EXPECT_EQ(plant.num_actuators(), 0);
  EXPECT_EQ(plant.num_constraints(), 0);
  EXPECT_EQ(plant.num_coupler_constraints(), 0);
  EXPECT_EQ(plant.num_distance_constraints(), 0);
  EXPECT_EQ(plant.num_ball_constraints(), 0);
  EXPECT_EQ(plant.num_force_elements(), 1);
}

GTEST_TEST(MultibodyPlantTest, EmptyWorldDiscrete) {
  const double discrete_update_period = 1.0e-3;
  MultibodyPlant<double> plant(discrete_update_period);
  DRAKE_EXPECT_THROWS_MESSAGE(plant.CreateDefaultContext(),
                              ".*CreateDefaultContext.*Finalize.*");
  plant.Finalize();
  EXPECT_EQ(plant.num_velocities(), 0);
  EXPECT_EQ(plant.num_positions(), 0);
  EXPECT_EQ(plant.num_multibody_states(), 0);
  EXPECT_EQ(plant.num_actuated_dofs(), 0);
  // Compute discrete update.
  auto context = plant.CreateDefaultContext();
  auto& discrete_state_vector = context->get_discrete_state_vector();
  EXPECT_EQ(discrete_state_vector.size(), 0);
  auto new_discrete_state = plant.AllocateDiscreteVariables();
  const systems::VectorBase<double>& new_discrete_state_vector =
      new_discrete_state->get_vector();
  EXPECT_EQ(new_discrete_state_vector.size(), 0);
  DRAKE_EXPECT_NO_THROW(plant.CalcForcedDiscreteVariableUpdate(
      *context, new_discrete_state.get()));
}

GTEST_TEST(MultibodyPlantTest, EmptyWorldContinuous) {
  MultibodyPlant<double> plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(plant.CreateDefaultContext(),
                              ".*CreateDefaultContext.*Finalize.*");
  plant.Finalize();
  EXPECT_EQ(plant.num_velocities(), 0);
  EXPECT_EQ(plant.num_positions(), 0);
  EXPECT_EQ(plant.num_multibody_states(), 0);
  EXPECT_EQ(plant.num_actuated_dofs(), 0);
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

GTEST_TEST(MultibodyPlantTest, EmptyWorldContactErrors) {
  MultibodyPlant<double> plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(
      MultibodyPlantTester::FindBodyByGeometryId(plant, GeometryId{}),
      ".*contact results.*invalid GeometryId.*");
  const GeometryId unknown_id = GeometryId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      MultibodyPlantTester::FindBodyByGeometryId(plant, unknown_id),
      ".*contact results.*ID is not known.*");
}

GTEST_TEST(ActuationPortsTest, CheckActuation) {
  // Create a MultibodyPlant consisting of two model instances, one actuated
  // and the other unactuated.
  MultibodyPlant<double> plant(0.0);
  const std::string acrobot_url =
      "package://drake/multibody/benchmarks/acrobot/acrobot.sdf";
  const std::string cylinder_url =
      "package://drake/multibody/benchmarks/free_body/"
      "uniform_solid_cylinder.urdf";
  Parser parser(&plant);
  auto acrobot_instance = parser.AddModelsFromUrl(acrobot_url).at(0);
  auto cylinder_instance = parser.AddModelsFromUrl(cylinder_url).at(0);
  plant.Finalize();

  // Verify the number of actuators.
  EXPECT_EQ(plant.num_actuated_dofs(acrobot_instance), 1);
  EXPECT_EQ(plant.num_actuated_dofs(cylinder_instance), 0);

  // Verify which bodies are free and modeled with quaternions.
  EXPECT_FALSE(plant.GetBodyByName("Link1").is_floating_base_body());
  EXPECT_FALSE(plant.GetBodyByName("Link1").has_quaternion_dofs());
  EXPECT_FALSE(plant.GetBodyByName("Link2").is_floating_base_body());
  EXPECT_FALSE(plant.GetBodyByName("Link2").has_quaternion_dofs());
  EXPECT_TRUE(
      plant.GetBodyByName("uniformSolidCylinder").is_floating_base_body());
  EXPECT_TRUE(
      plant.GetBodyByName("uniformSolidCylinder").has_quaternion_dofs());

  // Verify that we can get the actuation input ports.
  DRAKE_EXPECT_NO_THROW(plant.get_actuation_input_port());
  DRAKE_EXPECT_NO_THROW(plant.get_actuation_input_port(acrobot_instance));
  DRAKE_EXPECT_NO_THROW(plant.get_actuation_input_port(cylinder_instance));

  // Compute the derivatives without connecting the acrobot_instance port.
  // Actuation defaults to zero.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  std::unique_ptr<ContinuousState<double>> xdot_no_input =
      plant.AllocateTimeDerivatives();
  plant.CalcTimeDerivatives(*context, xdot_no_input.get());

  // Compute derivatives after fixing the acrobot actuation input port
  // explicitly to a zero value of actuation.
  plant.get_actuation_input_port(acrobot_instance).FixValue(context.get(), 0.0);
  std::unique_ptr<ContinuousState<double>> xdot_zero_input =
      plant.AllocateTimeDerivatives();
  plant.CalcTimeDerivatives(*context, xdot_zero_input.get());

  // Verify that both derivatives are the same since no input defaults to zero
  // actuation.
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(xdot_no_input->CopyToVector(),
                              xdot_zero_input->CopyToVector(), kEps,
                              MatrixCompareType::relative));

  // Verify that derivatives can be computed after fixing the cylinder actuation
  // input port with an empty vector.
  plant.get_actuation_input_port(cylinder_instance)
      .FixValue(context.get(), VectorXd(0));
  std::unique_ptr<ContinuousState<double>> xdot =
      plant.AllocateTimeDerivatives();
  plant.CalcTimeDerivatives(*context, xdot.get());

  // Non-zero actuation for the acrobot.
  plant.get_actuation_input_port(acrobot_instance).FixValue(context.get(), 5.0);
  plant.CalcTimeDerivatives(*context, xdot.get());

  // Distribute the actuation value of 5.0 between the two input ports.
  plant.get_actuation_input_port(acrobot_instance).FixValue(context.get(), 3.5);
  plant.get_actuation_input_port().FixValue(context.get(), 1.5);
  std::unique_ptr<ContinuousState<double>> xdot_sum =
      plant.AllocateTimeDerivatives();
  plant.CalcTimeDerivatives(*context, xdot_sum.get());

  // Verify that the contribution from per model instance actuation and full
  // plant actuation is additive.
  EXPECT_TRUE(CompareMatrices(xdot_sum->CopyToVector(), xdot->CopyToVector(),
                              kEps, MatrixCompareType::relative));
}

GTEST_TEST(MultibodyPlant, UniformGravityFieldElementTest) {
  MultibodyPlant<double> plant(0.0);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.AddForceElement<UniformGravityFieldElement>(Vector3d(-1, 0, 0)),
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
    const std::string url =
        "package://drake/multibody/benchmarks/acrobot/acrobot.sdf";
    std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, 0.0);
    const std::vector<ModelInstanceIndex> instances =
        Parser(plant_).AddModelsFromUrl(url);
    DRAKE_DEMAND(instances.size() == 1);
    model_instance_ = instances[0];

    // Sanity check on the availability of the optional source id before using
    // it.
    DRAKE_DEMAND(plant_->get_source_id() != std::nullopt);

    // Ensure that we can access the geometry ports pre-finalize.
    DRAKE_EXPECT_NO_THROW(plant_->get_geometry_query_input_port());
    DRAKE_EXPECT_NO_THROW(plant_->get_geometry_pose_output_port());

    DRAKE_EXPECT_THROWS_MESSAGE(
        plant_->get_state_output_port(),
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
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(*plant_, context_.get());

    ASSERT_GT(plant_->num_actuators(), 0);
    input_port_ =
        &plant_->get_actuation_input_port().FixValue(plant_context_, 0.0);
  }

  void SetUpDiscreteAcrobotPlant(double time_step, bool sampled) {
    systems::DiagramBuilder<double> builder;
    const std::string url =
        "package://drake/multibody/benchmarks/acrobot/acrobot.sdf";
    discrete_plant_ = std::make_unique<MultibodyPlant<double>>(time_step);
    discrete_plant_->SetUseSampledOutputPorts(sampled);
    Parser(discrete_plant_.get()).AddModelsFromUrl(url);
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

    EXPECT_TRUE(CompareMatrices(tau_g, tau_g_expected, kTolerance,
                                MatrixCompareType::relative));

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
      const RigidBody<double>& body = plant_->get_body(body_index);
      const SpatialForce<double>& F_Bo_W =
          body.GetForceInWorld(*plant_context_, forces);
      const double mass = body.default_mass();
      // TODO(amcastro-tri): provide RigidBody::EvalCOMInWorld().
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
  void VerifyCalcTimeDerivatives(double theta1, double theta2, double theta1dot,
                                 double theta2dot, double input_torque) {
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
    const Vector2d tau_damping(-parameters_.b1() * theta1dot,
                               -parameters_.b2() * theta2dot);

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

    EXPECT_TRUE(CompareMatrices(xdot, xdot_expected, kTolerance,
                                MatrixCompareType::relative));

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
    discrete_plant_->CalcForcedDiscreteVariableUpdate(*discrete_context_,
                                                      updates.get());

    // This test is verifying the discrete dynamics for our default discrete
    // solver only, currently SAP. Therefore we first verify this to be true.
    ASSERT_EQ(discrete_plant_->get_discrete_contact_solver(),
              DiscreteContactSolver::kSap);

    // Copies to plain Eigen vectors to verify the math.
    const int nv = plant_->num_velocities();
    const int nq = plant_->num_positions();
    const VectorXd x0 = context_->get_continuous_state_vector().CopyToVector();
    const VectorXd q0 = x0.segment(0, nq);
    const VectorXd v0 = x0.segment(nq, nv);
    const VectorXd xdot = derivatives_->CopyToVector();
    const VectorXd vdot = xdot.segment(nq, nv);
    const VectorXd xnext = updates->get_vector().CopyToVector();
    const VectorXd vnext = xnext.segment(nq, nv);

    // We verify the discrete update using the value of vdot computed using the
    // continuous model. However, we must take into consideration that SAP
    // evaluates the dissipation terms at the "next" state. Therefore the
    // "continuous" and "discrete" accelerations will differ by:
    //   a_sap = a0 - M₀⁻¹⋅D⋅(v−v₀)
    // we take this term into account below.
    const VectorXd& D = plant_->EvalJointDampingCache(*plant_context_);
    MatrixXd M = MatrixXd::Zero(2, 2);
    plant_->CalcMassMatrix(*plant_context_, &M);
    const VectorXd a_sap = vdot - M.ldlt().solve(D.asDiagonal() * (vnext - v0));

    // Verify that xnext is updated using a semi-explicit strategy, that is:
    //   vnext = v0 + dt * vdot
    //   qnext = q0 + dt * vnext
    VectorXd xnext_expected(plant_->num_multibody_states());
    const VectorXd vnext_expected = v0 + time_step * a_sap;
    xnext_expected.segment(nq, nv) = vnext_expected;
    // We use the fact that nq = nv for this case.
    xnext_expected.segment(0, nq) = q0 + time_step * vnext_expected;

    EXPECT_TRUE(CompareMatrices(xnext, xnext_expected, kTolerance,
                                MatrixCompareType::relative));
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
  const RigidBody<double>* link1_{nullptr};
  const RigidBody<double>* link2_{nullptr};
  RevoluteJoint<double>* shoulder_{nullptr};
  RevoluteJoint<double>* elbow_{nullptr};
  // Input port for the actuation:
  systems::FixedInputPortValue* input_port_{nullptr};
  // The model instance of the acrobot.
  ModelInstanceIndex model_instance_{};

  // Reference benchmark for verification.
  Acrobot<double> acrobot_benchmark_{Vector3d::UnitZ() /* Plane normal */,
                                     Vector3d::UnitY() /* Up vector */,
                                     parameters_.m1(),
                                     parameters_.m2(),
                                     parameters_.l1(),
                                     parameters_.l2(),
                                     parameters_.lc1(),
                                     parameters_.lc2(),
                                     parameters_.Ic1(),
                                     parameters_.Ic2(),
                                     parameters_.b1(),
                                     parameters_.b2(),
                                     parameters_.g()};
};

// Verifies we can compute the vector of generalized forces due to gravity on a
// model of an acrobot.
TEST_F(AcrobotPlantTests, VerifyCalcGravityGeneralizedForces) {
  // Some arbitrary values of non-zero state:
  VerifyCalcGravityGeneralizedForces(-M_PI / 5.0,
                                     M_PI / 2.0 /* joint's angles */);
  VerifyCalcGravityGeneralizedForces(M_PI / 3.0,
                                     -M_PI / 5.0 /* joint's angles */);
  VerifyCalcGravityGeneralizedForces(M_PI / 4.0,
                                     -M_PI / 3.0 /* joint's angles */);
  VerifyCalcGravityGeneralizedForces(-M_PI, -M_PI / 2.0 /* joint's angles */);
}

// Verifies the correctness of MultibodyPlant::CalcTimeDerivatives() on a model
// of an acrobot.
TEST_F(AcrobotPlantTests, CalcTimeDerivatives) {
  // Some random tests with non-zero state:
  VerifyCalcTimeDerivatives(-M_PI / 5.0, M_PI / 2.0, /* joint's angles */
                            0.5, 1.0,                /* joint's angular rates */
                            -1.0);                   /* Actuation torque */
  VerifyCalcTimeDerivatives(M_PI / 3.0, -M_PI / 5.0, /* joint's angles */
                            0.7, -1.0,               /* joint's angular rates */
                            1.0);                    /* Actuation torque */
  VerifyCalcTimeDerivatives(M_PI / 4.0, -M_PI / 3.0, /* joint's angles */
                            -0.5, 2.0,               /* joint's angular rates */
                            -1.5);                   /* Actuation torque */
  VerifyCalcTimeDerivatives(-M_PI, -M_PI / 2.0,      /* joint's angles */
                            -1.5, -2.5,              /* joint's angular rates */
                            2.0);                    /* Actuation torque */
}

// Verifies the correctness of MultibodyPlant::DoCalcDiscreteVariableUpdates()
// on a model of an acrobot.
TEST_F(AcrobotPlantTests, DoCalcDiscreteVariableUpdates) {
  // Set up an additional discrete state model of the same acrobot model.
  SetUpDiscreteAcrobotPlant(/* time_step = */ 0.001, /* sampled = */ false);

  const ModelInstanceIndex instance_index =
      discrete_plant_->GetModelInstanceByName("acrobot");

  // The generalized contact forces output port should have the same size as
  // number of generalized velocities in the model instance, even if there is
  // no contact geometry in the model.
  EXPECT_EQ(discrete_plant_
                ->get_generalized_contact_forces_output_port(instance_index)
                .size(),
            2);

  // Verify the implementation for a number of arbitrarily chosen states.
  VerifyDoCalcDiscreteVariableUpdates(-M_PI / 5.0,
                                      M_PI / 2.0, /* joint's angles */
                                      0.5, 1.0);  /* joint's angular rates */
  VerifyDoCalcDiscreteVariableUpdates(M_PI / 3.0,
                                      -M_PI / 5.0, /* joint's angles */
                                      0.7, -1.0);  /* joint's angular rates */
  VerifyDoCalcDiscreteVariableUpdates(M_PI / 4.0,
                                      -M_PI / 3.0, /* joint's angles */
                                      -0.5, 2.0);  /* joint's angular rates */
  VerifyDoCalcDiscreteVariableUpdates(-M_PI, -M_PI / 2.0, /* joint's angles */
                                      -1.5, -2.5); /* joint's angular rates */
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
  unique_ptr<systems::Context<double>> context = plant_->CreateDefaultContext();

  unique_ptr<AbstractValue> poses_value =
      plant_->get_geometry_pose_output_port().Allocate();
  DRAKE_EXPECT_NO_THROW(poses_value->get_value<FramePoseVector<double>>());
  const FramePoseVector<double>& poses =
      poses_value->get_value<FramePoseVector<double>>();

  // Compute the poses for each geometry in the model.
  plant_->get_geometry_pose_output_port().Calc(*context, poses_value.get());
  EXPECT_EQ(poses.size(), 2);  // Only two frames move.

  const FrameId world_frame_id =
      plant_->GetBodyFrameIdOrThrow(plant_->world_body().index());
  ASSERT_TRUE(plant_->GetBodyFromFrameId(world_frame_id) != nullptr);
  EXPECT_EQ(plant_->GetBodyFromFrameId(world_frame_id)->index(),
            plant_->world_body().index());
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  for (BodyIndex body_index(1); body_index < plant_->num_bodies();
       ++body_index) {
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
                                X_WB_expected.GetAsMatrix34(), kTolerance,
                                MatrixCompareType::relative));
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
      /* Verify this method is throwing for the right reasons. */
      "Body 'world' does not have geometry registered with it.");

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

TEST_F(AcrobotPlantTests, SetPositionWithNonFinites) {
  VectorX<double> p = VectorX<double>::Zero(plant_->num_positions());
  ASSERT_GT(p.rows(), 0);

  // First confirm we've got the right context and right size of things.
  EXPECT_NO_THROW(plant_->SetPositions(plant_context_, p));

  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    p[0] = bad;
    EXPECT_THROW(plant_->SetPositions(plant_context_, p), std::exception);
    EXPECT_THROW(plant_->SetPositions(plant_context_, model_instance_, p),
                 std::exception);
    EXPECT_THROW(plant_->SetPositions(*plant_context_,
                                      &plant_context_->get_mutable_state(),
                                      model_instance_, p),
                 std::exception);
  }
}

TEST_F(AcrobotPlantTests, SetDefaultPositionWithNonFinites) {
  VectorX<double> p = VectorX<double>::Zero(plant_->num_positions());
  ASSERT_GT(p.rows(), 0);

  // First confirm we've got the right size of things.
  EXPECT_NO_THROW(plant_->SetDefaultPositions(p));

  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    p[0] = bad;
    EXPECT_THROW(plant_->SetDefaultPositions(p), std::exception);
    EXPECT_THROW(plant_->SetDefaultPositions(model_instance_, p),
                 std::exception);
  }
}

TEST_F(AcrobotPlantTests, SetVelocitiesWithNonFinites) {
  VectorX<double> v = VectorX<double>::Zero(plant_->num_velocities());
  ASSERT_GT(v.rows(), 0);

  // First confirm we've got the right context and right size of things.
  EXPECT_NO_THROW(plant_->SetVelocities(plant_context_, v));

  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    v[0] = bad;
    EXPECT_THROW(plant_->SetVelocities(plant_context_, v), std::exception);
    EXPECT_THROW(plant_->SetVelocities(plant_context_, model_instance_, v),
                 std::exception);
    EXPECT_THROW(plant_->SetVelocities(*plant_context_,
                                       &plant_context_->get_mutable_state(),
                                       model_instance_, v),
                 std::exception);
  }
}

TEST_F(AcrobotPlantTests, SetVelocitiesInArrayWithNonFinites) {
  VectorX<double> v_all = VectorX<double>::Zero(plant_->num_velocities());
  ASSERT_GT(v_all.rows(), 0);
  VectorX<double> v_instance =
      VectorX<double>::Zero(plant_->num_velocities(model_instance_));

  // First confirm we've got the right context and right size of things.
  EXPECT_NO_THROW(
      plant_->SetVelocitiesInArray(model_instance_, v_instance, &v_all));

  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    v_instance[0] = bad;
    EXPECT_THROW(
        plant_->SetVelocitiesInArray(model_instance_, v_instance, &v_all),
        std::exception);
  }
}

TEST_F(AcrobotPlantTests, SetPositionAndVelocitiesWithNonFinites) {
  VectorX<double> q = VectorX<double>::Zero(plant_->num_multibody_states());
  ASSERT_GT(q.rows(), 0);

  // First confirm we've got the right context and right size of things.
  EXPECT_NO_THROW(plant_->SetPositionsAndVelocities(plant_context_, q));

  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity()}) {
    q[0] = bad;
    EXPECT_THROW(plant_->SetPositionsAndVelocities(plant_context_, q),
                 std::exception);
    EXPECT_THROW(
        plant_->SetPositionsAndVelocities(plant_context_, model_instance_, q),
        std::exception);
  }
}

GTEST_TEST(MultibodyPlantTest, SetDefaultFloatingBaseBodyPose) {
  // We cannot use Acrobot for testing `SetDefaultFloatingBaseBodyPose` since it
  // has no free bodies.
  MultibodyPlant<double> plant(0.0);
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const auto& body =
      plant.AddRigidBody("body", SpatialInertia<double>::MakeUnitary());
  const auto& welded_body =
      plant.AddRigidBody("welded body", SpatialInertia<double>::MakeUnitary());
  plant.WeldFrames(plant.world_body().body_frame(), welded_body.body_frame());
  // Default pose is identity when unset.
  EXPECT_TRUE(
      CompareMatrices(plant.GetDefaultFloatingBaseBodyPose(body).GetAsMatrix4(),
                      RigidTransformd::Identity().GetAsMatrix4()));

  // Ok to set default pose for any body pre-finalize.
  const RigidTransformd X_WB(RollPitchYawd(0.1, 0.2, 0.3), Vector3d(1, 2, 3));
  EXPECT_NO_THROW(plant.SetDefaultFloatingBaseBodyPose(body, X_WB));
  EXPECT_NO_THROW(plant.SetDefaultFloatingBaseBodyPose(welded_body, X_WB));
  const double kTolerance = 4.0 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(
      CompareMatrices(plant.GetDefaultFloatingBaseBodyPose(body).GetAsMatrix4(),
                      X_WB.GetAsMatrix4(), kTolerance));
  EXPECT_TRUE(CompareMatrices(
      plant.GetDefaultFloatingBaseBodyPose(welded_body).GetAsMatrix4(),
      X_WB.GetAsMatrix4(), kTolerance));

  plant.Finalize();
  EXPECT_GT(plant.num_positions(), 0);
  auto context = plant.CreateDefaultContext();
  EXPECT_TRUE(CompareMatrices(body.EvalPoseInWorld(*context).GetAsMatrix4(),
                              X_WB.GetAsMatrix4(), kTolerance));
  // The pose of a non-free body isn't affected by the call to
  // SetDefaultFloatingBaseBodyPose() even though it's allowed.
  EXPECT_TRUE(
      CompareMatrices(welded_body.EvalPoseInWorld(*context).GetAsMatrix4(),
                      RigidTransformd::Identity().GetAsMatrix4(), kTolerance));
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
  shoulder_->set_random_angle_distribution(M_PI + 0.02 * gaussian(generator));
  elbow_->set_random_angle_distribution(0.05 * gaussian(generator));

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
  const std::string acrobot_url =
      "package://drake/multibody/benchmarks/acrobot/acrobot.sdf";
  const std::string cylinder_url =
      "package://drake/multibody/benchmarks/free_body/"
      "uniform_solid_cylinder.urdf";
  Parser(&plant).AddModelsFromUrl(acrobot_url);
  Parser(&plant).AddModelsFromUrl(cylinder_url);
  Parser(&plant, "cylinder2").AddModelsFromUrl(cylinder_url);

  plant.set_name("MyTestMBP");
  const std::string dot = std::regex_replace(plant.GetTopologyGraphvizString(),
                                             std::regex("\\n"), "");

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
  EXPECT_NE(std::string::npos, dot.find("cylinder2::uniformSolidCylinder"))
      << dot;
  // Check for the Acrobot elbow joint.
  EXPECT_NE(std::string::npos, dot.find("ElbowJoint [revolute]")) << dot;

  // Finalize() will add floating joints for the free bodies.
  plant.Finalize();
  const std::string dot_finalized = std::regex_replace(
      plant.GetTopologyGraphvizString(), std::regex("\\n"), "");

  // Check that the pre-finalize string is a substring of the post-finalize
  // string (ignoring newlines and the ending '}' as the last character)
  EXPECT_THAT(dot_finalized.c_str(),
              testing::HasSubstr(dot.substr(0, dot.size() - 1).c_str()));

  // Check that the two floating joints created at Finalize() exist.
  const size_t pos_finalized =
      dot_finalized.find("uniformSolidCylinder [quaternion_floating]");
  EXPECT_NE(std::string::npos, pos_finalized) << dot_finalized;
  EXPECT_NE(std::string::npos,
            dot_finalized.find("uniformSolidCylinder [quaternion_floating]",
                               pos_finalized + 1));
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
// The optional constructor parameter @p weld_to_next will cause selected
// joints to be welds instead of revolute joints. The parameter is a bitmap
// where weld_to_next[k] specifies a weld between sphere(k) and sphere(k+1) if
// true. Otherwise, it specifies a revolute joint between those
// bodies. Regardless of the size of vector passed to @p weld_to_next, it will
// be resized internally, filled with zeros (revolute joint requests) if
// necessary.
//
// Specifying welds joints will reduce the number of collisions, thanks to
// automatic filtering of welded subgraphs.
class SphereChainScenario {
 public:
  explicit SphereChainScenario(int sphere_count,
                               std::vector<bool> weld_to_next = {})
      : sphere_count_(sphere_count) {
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
      // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
      const RigidBody<double>& sphere = plant_->AddRigidBody(
          "Sphere" + to_string(i), SpatialInertia<double>::MakeUnitary());
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
      const auto& [body, id] = make_sphere(i);
      spheres_.push_back(body);
      sphere_ids_.push_back(id);
    }
    // Add hinges xor welds between spheres.
    weld_to_next.resize(sphere_count_ - 1);
    for (int i = 0; i < sphere_count_ - 1; ++i) {
      if (weld_to_next[i]) {
        plant_->WeldFrames(spheres_[i]->body_frame(),
                           spheres_[i + 1]->body_frame());
      } else {
        plant_->AddJoint<RevoluteJoint>(
            "hinge" + to_string(i) + "_" + to_string(i + 1), *spheres_[i],
            std::nullopt, *spheres_[i + 1], std::nullopt, Vector3d::UnitY());
      }
    }

    // Body with no registered frame.
    // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
    no_geometry_body_ = &plant_->AddRigidBody(
        "NothingRegistered", SpatialInertia<double>::MakeUnitary());
  }

  void Finalize() {
    // We are done defining the model.
    plant_->Finalize();

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
    const auto& query_object =
        plant_->get_geometry_query_input_port()
            .Eval<geometry::QueryObject<double>>(*plant_context_);
    return query_object.ComputePointPairPenetration();
  }

  // Get all bodies of the internal plant.
  std::vector<const RigidBody<double>*> get_all_bodies() const {
    std::vector<const RigidBody<double>*> all_bodies;
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
  const RigidBody<double>& body1 =
      plant.AddRigidBody("body1", SpatialInertia<double>::NaN());
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetBodyFrameIdOrThrow(body1.index()),
      "Body 'body1' does not have geometry registered with it.");

  geometry::SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  ASSERT_TRUE(plant.geometry_source_is_registered());

  // Now that the plant has been registered as a source, old bodies have been
  // updated with FrameIds.
  DRAKE_EXPECT_NO_THROW(plant.GetBodyFrameIdOrThrow(body1.index()));

  // And new bodies have FrameIds immediately upon creation.
  const RigidBody<double>& body2 =
      plant.AddRigidBody("body2", SpatialInertia<double>::NaN());
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
  for (bool do_filters : {false, true}) {
    SphereChainScenario scenario(3);
    scenario.mutable_plant()->set_adjacent_bodies_collision_filters(do_filters);
    scenario.Finalize();
    std::vector<geometry::PenetrationAsPointPair<double>> contacts =
        scenario.ComputePointPairPenetration();

    // The expected collisions.
    const std::set<std::pair<GeometryId, GeometryId>>& expected_pairs =
        scenario.unfiltered_collisions();
    if (do_filters) {
      ASSERT_EQ(contacts.size(), expected_pairs.size());
      auto expect_pair_in_set = [&expected_pairs](GeometryId id1,
                                                  GeometryId id2) {
        auto pair1 = std::make_pair(id1, id2);
        auto pair2 = std::make_pair(id2, id1);
        if (!expected_pairs.contains(pair1) &&
            !expected_pairs.contains(pair2)) {
          GTEST_FAIL() << fmt::format(
              "The pair {}, {} is not in the expected set", id1, id2);
        }
      };
      for (int i = 0; i < static_cast<int>(contacts.size()); ++i) {
        const auto& point_pair = contacts[i];
        expect_pair_in_set(point_pair.id_A, point_pair.id_B);
      }
    } else {
      ASSERT_GT(contacts.size(), expected_pairs.size());
    }
  }
}

// Ensure world-welded bodies are collision filtered; see also #11116.
GTEST_TEST(MultibodyPlantTest, FilterWorldWelds) {
  SphereChainScenario scenario(3, {1, 1, 1});
  scenario.mutable_plant()->WeldFrames(
      scenario.mutable_plant()->world_body().body_frame(),
      scenario.sphere(0).body_frame());
  scenario.Finalize();
  std::vector<geometry::PenetrationAsPointPair<double>> contacts =
      scenario.ComputePointPairPenetration();

  // The actual collisions should be 0; everything is welded to the world.
  ASSERT_EQ(contacts.size(), 0);
}

// Ensure that all welded subgraphs are collision filtered.
GTEST_TEST(MultibodyPlantTest, FilterWeldedSubgraphs) {
  // Build a chain with two welded subgraphs; a chain of 3, followed later by a
  // chain of 4.
  SphereChainScenario scenario(12, {0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1});
  scenario.Finalize();
  std::vector<geometry::PenetrationAsPointPair<double>> contacts =
      scenario.ComputePointPairPenetration();

  // The actual collisions should be fewer than the expected collisions, by
  // exactly the number of edges within the welded subgraphs.
  const std::set<std::pair<GeometryId, GeometryId>>& expected_pairs =
      scenario.unfiltered_collisions();
  // The numbers of collisions filtered within each subgroup, which are just
  // the binomial coefficients C(3,2) = 3 and C(4,2) = 6.
  constexpr int filtered_in_first_subgraph = 3;
  constexpr int filtered_in_second_subgraph = 6;
  constexpr int filtered =
      filtered_in_first_subgraph + filtered_in_second_subgraph;
  ASSERT_EQ(contacts.size(), expected_pairs.size() - filtered);
}

// Tests the error conditions for CollectRegisteredGeometries.
GTEST_TEST(MultibodyPlantTest, CollectRegisteredGeometriesErrors) {
  MultibodyPlant<double> plant(0.0);
  const RigidBody<double>& body =
      plant.AddRigidBody("body", SpatialInertia<double>::MakeUnitary());

  // It's an error to call this without a SceneGraph.
  DRAKE_EXPECT_THROWS_MESSAGE(plant.CollectRegisteredGeometries({&body}),
                              ".*geometry_source_is_registered.*failed.*");

  // With a scene graph, it passes.
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  EXPECT_NO_THROW(plant.CollectRegisteredGeometries({&body}));
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
      GeometrySet set = plant.CollectRegisteredGeometries(
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
  const std::string sdf_url =
      "package://drake/multibody/plant/test/split_pendulum.sdf";
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelsFromUrl(sdf_url);
  const RigidBody<double>& upper = plant.GetBodyByName("upper_section");
  const RigidBody<double>& lower = plant.GetBodyByName("lower_section");

  // Add a new body, and weld it using `WeldFrames` (to ensure that topology is
  // updated via this API).
  const RigidBody<double>& extra = plant.AddRigidBody(
      "extra", default_model_instance(), SpatialInertia<double>::NaN());
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
  const RigidBody<AutoDiffXd>& upper_ad =
      plant_ad->GetBodyByName("upper_section");
  const RigidBody<AutoDiffXd>& lower_ad =
      plant_ad->GetBodyByName("lower_section");
  const RigidBody<AutoDiffXd>& extra_ad = plant_ad->GetBodyByName("extra");

  EXPECT_THAT(plant_ad->GetBodiesWeldedTo(plant_ad->world_body()),
              UnorderedElementsAre(&plant_ad->world_body(), &extra_ad));
  EXPECT_THAT(plant_ad->GetBodiesWeldedTo(lower_ad),
              UnorderedElementsAre(&upper_ad, &lower_ad));
}

// Checks plumbing and top-level contract in MultibodyPlant.
// See more complete testing of this graph query in `multibody_graph_test`.
GTEST_TEST(MultibodyPlantTest, GetBodiesKinematicallyAffectedBy) {
  // This test expects that the following model has a world body and a pair of
  // welded-together bodies.
  const std::string sdf_url =
      "package://drake/multibody/plant/test/split_pendulum.sdf";
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelsFromUrl(sdf_url);
  const RigidBody<double>& upper = plant.GetBodyByName("upper_section");
  const RigidBody<double>& lower = plant.GetBodyByName("lower_section");
  const JointIndex shoulder = plant.GetJointByName("pin").index();
  const JointIndex elbow = plant.GetJointByName("weld").index();
  // Add a new body, and weld it to the world body.
  const RigidBody<double>& extra = plant.AddRigidBody(
      "extra", default_model_instance(), SpatialInertia<double>::NaN());
  plant.WeldFrames(plant.world_frame(), extra.body_frame());

  const std::vector<JointIndex> joints1{shoulder};
  std::vector<BodyIndex> expected_bodies1{lower.index(), upper.index()};
  std::sort(expected_bodies1.begin(), expected_bodies1.end());

  // Verify we can only call GetBodiesKinematicallyAffectedBy() post-finalize.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetBodiesKinematicallyAffectedBy(joints1),
      "Pre-finalize calls to .*GetBodiesKinematicallyAffectedBy.*");
  plant.Finalize();
  EXPECT_EQ(plant.GetBodiesKinematicallyAffectedBy(joints1), expected_bodies1);

  // Adding a weld joint doesn't change the result; still the same bodies
  // based on the non-weld joints.
  const std::vector<JointIndex> joints2{shoulder, elbow};
  EXPECT_EQ(plant.GetBodiesKinematicallyAffectedBy(joints2), expected_bodies1);

  // Passing only a weld joint produces no bodies.
  EXPECT_TRUE(plant.GetBodiesKinematicallyAffectedBy({elbow}).empty());

  // Test throw condition: unregistered joint.
  std::vector<JointIndex> joint100{JointIndex(100)};
  DRAKE_EXPECT_THROWS_MESSAGE(plant.GetBodiesKinematicallyAffectedBy(joint100),
                              ".*No joint with index.*registered.*removed.");
}

// Weld a body to World, but with the body as the parent and World as the child.
// This is simple for a Drake user, but for Drake internal developers, this body
// (parent)-to-World (child) weld is implemented with a reversed Weld mobilizer
// that specifies inboard-to-outboard order as World-to-body. We'll do several
// tests here, in increasing difficulty of hand-computing the right answers, and
// we'll include a forward (World as parent) case that shows how the results
// must change between that and the reverse case. Reaction forces are the tricky
// part since we are required to report them in the _joint's_ child frame (in
// this case a frame on World) but internally they are calculated on the
// outboard body of the mobilizer.
GTEST_TEST(MultibodyPlantTest, ReversedWeldJoint) {
  // We'll be looking for large changes: sign reversal, shift by a meter,
  // rotate 90°. Just need to avoid roundoff troubles.
  constexpr double kTolerance = 16 * std::numeric_limits<double>::epsilon();
  const double g = UniformGravityFieldElement<double>::kDefaultStrength;
  MultibodyPlant<double> plant(0.0);

  // Coincident frames test: We expect the sign of the reaction force to account
  // for the possibility that a Drake user may create a "weird" weld joint with
  // parent=body, child=World (or in general, any case where the parent ends up
  // more-outboard than the child after toplogical analysis). Internally, the
  // mobilizer stores this with more sensible topological order, i.e.,
  // inboard=World and outboard=body.
  const RigidBody<double>& parent_body_coincident =
      plant.AddRigidBody("parent_body_coincident", default_model_instance(),
                         SpatialInertia<double>::MakeUnitary());
  // Create a reverse weld (parent=body, child=World) in which the all the
  // transforms are identity, i.e., X_PJp, X_CJc, X_JpJc are all identity.
  const RigidTransform<double> X_Identity;
  const Joint<double>& reverse_weld_coincident = plant.AddJoint<WeldJoint>(
      "reverse_weld_coincident", parent_body_coincident, X_Identity,
      plant.world_body(), X_Identity, X_Identity);

  // General test: Uses all non-identity rotations and non-zero shifts to ensure
  // reaction torque/forces get properly re-expressed and shifted for the
  // reverse case. See multibody_plant_test_reverse_weld.png in this test
  // directory for the 2D schematic for this test. The 2D schematic has (from
  // left-to-right) parent body frame P, joint frame Jp, joint frame Jc, child
  // body frame C. Each of the frame origins are 1 meter apart along a
  // horizontal line that is parallel to the unit vector Px (fixed in frame P).
  // Hence, the position vectors locating the frame origins are p_PJp = Px,
  // p_JpJc = Px, p_JcC = Px. The orientation of frame Jp in frame P is
  // characterized by a right-hand rotation of π/4 about Py (fixed in frame P).
  // Similarly frame Jc in Jp is π/4 about Py, and frame C in Jc is π/2 about
  // Py. Hence the "y" basis vector of each frame is in the same direction (Py =
  // Jpy = Jcy = Cy). Note: Frames translate in the Px direction whereas frames
  // rotate about Py to simplify by-hand calculations, but not to so simple
  // (e.g., with both translation and rotation in the Px direction) so as to
  // hide frame problems. The 2D schematic shows the unit vector Pz (fixed in
  // frame P) as vertically upward, which is opposite gravity when P=World
  // (typical case). However, Pz is in the gravity direction when P=child
  // (reverse case).

  // Rotation matrix relating frames.
  const RotationMatrixd R_PJp(RollPitchYawd(0, M_PI / 4, 0));
  const RotationMatrixd R_JpJc(RollPitchYawd(0, M_PI / 4, 0));
  const RotationMatrixd R_JcC(RollPitchYawd(0, M_PI / 2, 0));

  // Position vectors relating frame origins are a unit vector apart.
  const Vector3d Px_P(1, 0, 0);  // Px unit vector (expressed in frame P).
  const RotationMatrixd R_JpP = R_PJp.inverse();
  const Vector3d Px_Jp = R_JpP * Px_P;  // Px unit vector (expressed in Jp).
  const RotationMatrixd R_JcJp = R_JpJc.inverse();
  const Vector3d Px_Jc = R_JcJp * Px_Jp;  // Px unit vector, expressed in Jc.

  // Rigid transforms relating frames.
  const RigidTransformd X_PJp(R_PJp, Px_P);
  const RigidTransformd X_JpJc(R_JpJc, Px_Jp);
  const RigidTransformd X_JcC(R_JcC, Px_Jc);
  const RigidTransformd X_CJc = X_JcC.inverse();

  // Weld a body with parent=World, child=child_body_general. Mobilizer
  // inboard/outboard ordering will match (inboard=World, outboard=child). Joint
  // reaction torque/force is reported on child body at Jc, expressed in Jc.
  const RigidBody<double>& child_body_general =
      plant.AddRigidBody("child_body_general", default_model_instance(),
                         SpatialInertia<double>::MakeUnitary());
  auto& typical_weld_general =
      plant.AddJoint<WeldJoint>("typical_weld_general", plant.world_body(),
                                X_PJp, child_body_general, X_CJc, X_JpJc);

  // Weld a body with parent=parent_body_general, child=World. Mobilizer
  // inboard/outboard ordering is reversed (inboard=parent, outboard=World).
  // Joint reaction torque/force is reported on World at Jc (fixed to World).
  const RigidBody<double>& reverse_body_general =
      plant.AddRigidBody("reverse_body_general", default_model_instance(),
                         SpatialInertia<double>::MakeUnitary());
  auto& reverse_weld_general =
      plant.AddJoint<WeldJoint>("reverse_weld_general", reverse_body_general,
                                X_PJp, plant.world_body(), X_CJc, X_JpJc);

  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  // This port returns F_CJc_Jc reactions on each joint's child body, indexed
  // by joint ordinal.
  const std::vector<SpatialForce<double>>& reaction_forces =
      plant.get_reaction_forces_output_port()
          .Eval<std::vector<SpatialForce<double>>>(*context);

  // Coincident test case with reversed weld: Since child = World, the force on
  // World at Jc (which is coincident will all other frames), expressed in Jc
  // is a downward force of m*g (with mass m = 1 kg).
  Vector3d torque_expected(0, 0, 0), force_expected(0, 0, -g);
  EXPECT_TRUE(CompareMatrices(
      reaction_forces[reverse_weld_coincident.ordinal()].get_coeffs(),
      SpatialForce<double>(torque_expected, force_expected).get_coeffs(),
      kTolerance));

  // General test with typical weld (parent = World, child = body). After
  // drawing the 2D schematic (see system description above), it is clear that
  // Jc's +x unit vector points in the World's -z direction. The joint reaction
  // force on the child body is m*g in the direction of Jc's -x unit vector.
  // The joint reaction torque on the child body at Jc is calculated in view of
  // the gravity force m*g (with mass m = 1 kg) acting at Co (child body C's
  // center of mass), which is 1 meter from Jc.
  torque_expected = Vector3d(0, -g, 0);
  force_expected = Vector3d(-g, 0, 0);
  EXPECT_TRUE(CompareMatrices(
      reaction_forces[typical_weld_general.ordinal()].get_coeffs(),
      SpatialForce<double>(torque_expected, force_expected).get_coeffs(),
      kTolerance));

  // General test with reversed weld (parent = body, child = World). After
  // viewing the 2D schematic (see description above), it is clear that Jc's +x
  // unit vector points in the World's +z direction. To calculate the joint
  // force on child=World at Jc, it helps to see that the child's force on joint
  // & parent=body at Jc is m*g in Jc's +x direction. By action/reaction the
  // joint force on the child=World at Jc is m*g in Jc's -x direction. To
  // calculate the joint torque on the child=World at Jc, it helps to see that
  // the child's torque on joint & parent=body at Jc arises from a 2 meter
  // moment arm that yields 2*m*g in Jc's -y direction. By action/reaction, the
  // joint torque on the child=World at Jc is 2*m*g in Jc's +y direction.
  torque_expected = Vector3d(0, 2 * g, 0);
  force_expected = Vector3d(-g, 0, 0);
  EXPECT_TRUE(CompareMatrices(
      reaction_forces[reverse_weld_general.ordinal()].get_coeffs(),
      SpatialForce<double>(torque_expected, force_expected).get_coeffs(),
      kTolerance));
}

GTEST_TEST(MultibodyPlantTest, ReversedRevoluteJoint) {
  const double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  MultibodyPlant<double> plant(0.0);

  // Add a normal body which we'll use as the child of a revolute joint.
  const RigidBody<double>& body = plant.AddRigidBody(
      "body", default_model_instance(), SpatialInertia<double>::MakeUnitary());
  const RevoluteJoint<double>& revolute = plant.AddJoint<RevoluteJoint>(
      "revolute", plant.world_body(), {}, body, {}, Vector3d(1, 1, 1));

  // Add a body with parent=reverse_body, child=world.
  const RigidBody<double>& reverse_body =
      plant.AddRigidBody("reverse_body", default_model_instance(),
                         SpatialInertia<double>::MakeUnitary());
  const RevoluteJoint<double>& reverse_revolute =
      plant.AddJoint<RevoluteJoint>("reverse_revolute", reverse_body, {},
                                    plant.world_body(), {}, Vector3d(1, 1, 1));
  EXPECT_NO_THROW(plant.Finalize());
  auto context = plant.CreateDefaultContext();

  for (int i = 0; i < 3; ++i) {
    const double third = std::numbers::inv_sqrt3_v<double>;
    EXPECT_NEAR(revolute.revolute_axis()[i], third, kTolerance);
    EXPECT_NEAR(reverse_revolute.revolute_axis()[i], third, kTolerance);
  }

  // The forward and reverse joints should produce the same motion, but the
  // meaning of the generalized coordinate q (angle) is reversed.
  revolute.set_angle(&*context, 0.125);
  EXPECT_EQ(revolute.get_angle(*context), 0.125);
  reverse_revolute.set_angle(&*context, -0.125);
  EXPECT_EQ(reverse_revolute.get_angle(*context), -0.125);

  const RigidTransformd pose = body.EvalPoseInWorld(*context);
  const RigidTransformd rpose = reverse_body.EvalPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(rpose.GetAsMatrix34(), pose.GetAsMatrix34(),
                              kTolerance, MatrixCompareType::relative));

  // Now check the velocities.
  revolute.set_angular_rate(&*context, 1.5);
  EXPECT_EQ(revolute.get_angular_rate(*context), 1.5);
  reverse_revolute.set_angular_rate(&*context, -1.5);
  EXPECT_EQ(reverse_revolute.get_angular_rate(*context), -1.5);
  const SpatialVelocity<double>& velocity =
      body.EvalSpatialVelocityInWorld(*context);
  const SpatialVelocity<double>& rvelocity =
      reverse_body.EvalSpatialVelocityInWorld(*context);
  EXPECT_TRUE(CompareMatrices(rvelocity.get_coeffs(), velocity.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(MultibodyPlantTest, ReversedPrismaticJoint) {
  const double kTolerance = 4 * std::numeric_limits<double>::epsilon();
  MultibodyPlant<double> plant(0.0);

  // Add a normal body which we'll use as the child of a prismatic joint.
  const RigidBody<double>& body = plant.AddRigidBody(
      "body", default_model_instance(), SpatialInertia<double>::MakeUnitary());
  const PrismaticJoint<double>& prismatic = plant.AddJoint<PrismaticJoint>(
      "prismatic", plant.world_body(), {}, body, {}, Vector3d(1, 1, 1));

  // Add a body ("reverse_body") and a joint ("reverse_prismatic") with
  // parent=reverse_body, child=world.
  const RigidBody<double>& reverse_body =
      plant.AddRigidBody("reverse_body", default_model_instance(),
                         SpatialInertia<double>::MakeUnitary());
  const PrismaticJoint<double>& reverse_prismatic =
      plant.AddJoint<PrismaticJoint>("reverse_prismatic", reverse_body, {},
                                     plant.world_body(), {}, Vector3d(1, 1, 1));
  EXPECT_NO_THROW(plant.Finalize());
  auto context = plant.CreateDefaultContext();

  for (int i = 0; i < 3; ++i) {
    const double third = std::numbers::inv_sqrt3_v<double>;
    EXPECT_NEAR(prismatic.translation_axis()[i], third, kTolerance);
    EXPECT_NEAR(reverse_prismatic.translation_axis()[i], third, kTolerance);
  }

  // The forward and reverse joints should produce the same motion, but the
  // meaning of the generalized coordinate q (translation) is reversed.
  prismatic.set_translation(&*context, 0.125);
  EXPECT_EQ(prismatic.get_translation(*context), 0.125);
  reverse_prismatic.set_translation(&*context, -0.125);
  EXPECT_EQ(reverse_prismatic.get_translation(*context), -0.125);

  const RigidTransformd pose = body.EvalPoseInWorld(*context);
  const RigidTransformd rpose = reverse_body.EvalPoseInWorld(*context);
  EXPECT_TRUE(CompareMatrices(rpose.GetAsMatrix34(), pose.GetAsMatrix34(),
                              kTolerance, MatrixCompareType::relative));

  // Now check the velocities.
  prismatic.set_translation_rate(&*context, 1.5);
  EXPECT_EQ(prismatic.get_translation_rate(*context), 1.5);
  reverse_prismatic.set_translation_rate(&*context, -1.5);
  EXPECT_EQ(reverse_prismatic.get_translation_rate(*context), -1.5);
  const SpatialVelocity<double>& velocity =
      body.EvalSpatialVelocityInWorld(*context);
  const SpatialVelocity<double>& rvelocity =
      reverse_body.EvalSpatialVelocityInWorld(*context);
  EXPECT_TRUE(CompareMatrices(rvelocity.get_coeffs(), velocity.get_coeffs(),
                              kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(MultibodyPlantTest, UnsupportedReversedJoint) {
  MultibodyPlant<double> plant(0.0);

  // Add a body ("reverse_body") and a joint ("reverse_universal") with
  // parent=reverse_body, child=world. Reversal is allowed for a limited set of
  // joints, and UniversalJoint is not currently among them.
  const RigidBody<double>& reverse_body =
      plant.AddRigidBody("reverse_body", default_model_instance(),
                         SpatialInertia<double>::MakeUnitary());
  plant.AddJoint<UniversalJoint>("reverse_universal", reverse_body, {},
                                 plant.world_body(), {});

  // Check that the message (a) identifies Finalize() as the failed operation,
  // (b) complains about the non-reversible joint, (c) lists the reversible
  // ones (in alphabetical order), and (e) provides instructions about what to
  // do to get around the problem.
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.Finalize(),
      ".*Finalize.*parent/child ordering.*universal joint reverse_universal"
      ".*reversed.*does not support.*universal.*can be reversed.*"
      ".*prismatic.*revolute.*weld.*Reverse.*ordering.*");
}

// This test verifies that these two issues are fixed:
//  - #9939 (duplicate welds give a bad error message)
//  - #17429 (can't weld anything but base link)
//
// Issue #17429 complained that welding a non-base body to World failed.
// This is a model of the system shown in that issue:
//     base -p-> waist -r-> waist1 -r-> waist2 -r-> torso -r-> arm
//       legend: p=prismatic, r=revolute, parent -> child
// When "base" is welded to World, the parent->child directions are preserved
// as inboard->outboard directions in the tree. But if we weld "waist" or
// "torso" to World then some of the tree's mobilizers have to be reversed from
// the joints. That should work as of PR #22949 (2025-04-30).
GTEST_TEST(MultibodyPlantTest, WeldOfNonBaseBody) {
  auto fill_plant = [](MultibodyPlant<double>* plant) {
    const auto& base = plant->AddRigidBody("base");
    const auto& waist = plant->AddRigidBody("waist");
    const auto& waist1 = plant->AddRigidBody("waist1");
    const auto& waist2 = plant->AddRigidBody("waist2");
    const auto& torso = plant->AddRigidBody("torso");
    const auto& arm = plant->AddRigidBody("arm");

    plant->AddJoint<PrismaticJoint>("prismatic_z", base, {}, waist, {},
                                    Vector3d(0, 0, 1));
    plant->AddJoint<RevoluteJoint>("torso_joint1", waist, {}, waist1, {},
                                   Vector3d(1, 0, 0));
    plant->AddJoint<RevoluteJoint>("torso_joint2", waist1, {}, waist2, {},
                                   Vector3d(0, 1, 0));
    plant->AddJoint<RevoluteJoint>("torso_joint3", waist2, {}, torso, {},
                                   Vector3d(0, 1, 0));
    plant->AddJoint<RevoluteJoint>("shoulder", torso, {}, arm, {},
                                   Vector3d(1, 1, 1));
  };

  // Issue #17429, first with no welds so the robot is floating.
  MultibodyPlant<double> floating(0.0);
  fill_plant(&floating);
  EXPECT_NO_THROW(floating.Finalize());

  // Issue #17429, the three welded cases mentioned above.
  for (auto body : {"base", "waist", "torso"}) {
    MultibodyPlant<double> plant(0.0);
    fill_plant(&plant);
    plant.AddJoint<WeldJoint>("body_to_world", plant.world_body(), {},
                              plant.GetRigidBodyByName(body), {},
                              RigidTransformd());
    EXPECT_NO_THROW(plant.Finalize());
  }

  // Issue #9939, verify that welding the same body twice now produces a
  // reasonable error message.
  MultibodyPlant<double> bad_double_weld(0.0);
  fill_plant(&bad_double_weld);
  bad_double_weld.AddJoint<WeldJoint>(
      "base_to_world1", bad_double_weld.world_body(), {},
      bad_double_weld.GetRigidBodyByName("base"), {}, RigidTransformd());

  DRAKE_EXPECT_THROWS_MESSAGE(
      bad_double_weld.AddJoint<WeldJoint>(
          "base_to_world2", bad_double_weld.world_body(), {},
          bad_double_weld.GetRigidBodyByName("base"), {}, RigidTransformd()),
      "AddJoint.*already.*base_to_world1.*world.*base.*base_to_world2.*"
      "not allowed.*");

  // The attempt to add the redundant joint should have been ignored.
  EXPECT_NO_THROW(bad_double_weld.Finalize());
}

// Currently we don't support automatic modeling of systems where the links
// and joints form topological loops. Make sure we reject those for now.
GTEST_TEST(MultibodyPlantTest, UnsupportedTopologicalLoop) {
  MultibodyPlant<double> plant(0.0);

  // Create a loop with two bodies:
  //   World->body1->body2<-World
  const RigidBody<double>& body1 = plant.AddRigidBody(
      "body1", default_model_instance(), SpatialInertia<double>::MakeUnitary());
  const RigidBody<double>& body2 = plant.AddRigidBody(
      "body2", default_model_instance(), SpatialInertia<double>::MakeUnitary());
  plant.AddJoint<RevoluteJoint>("joint1", plant.world_body(), {}, body1, {},
                                Vector3d(0, 0, 1));
  plant.AddJoint<RevoluteJoint>("joint2", body1, {}, body2, {},
                                Vector3d(0, 0, 1));
  plant.AddJoint<RevoluteJoint>("joint3", plant.world_body(), {}, body2, {},
                                Vector3d(0, 0, 1));

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.Finalize(),
      "The bodies and joints of this system form one or more loops.*");
}

// Position kinematics attempts to optimize for cases when X_PF or X_MB are
// identity matrices. That makes for four distinct cases which we'll engineer
// here and check that we get X_PB = X_PF * X_FM * X_MB in all cases. We'll
// avoid any special handling of World by making four systems like this:
//    World -> parent => child
// with the joint "=>" being the one we'll check. We don't have direct control
// over F and M since we only get to specify Jp and Jc for a joint so we'll
// check to make sure we're testing the right cases. It's sufficient to use
// easy-to-calculate translations here (no rotations) since we just want to
// see if the right transforms got applied.
GTEST_TEST(MultibodyTree, PositionKinematicsFrameOptimizations) {
  // Test one case, see CalcPositionKinematicsCache_BaseToTip() in
  // body_node_impl.cc for correspondence.
  auto test_case = [](int which_case) {
    const bool X_PF_is_identity = which_case & 0b01;
    const bool X_BM_is_identity = which_case & 0b10;

    const RigidTransformd X_PJp = X_PF_is_identity
                                      ? RigidTransformd()
                                      : RigidTransformd(Vector3d(10, 11, 12));

    const RigidTransformd X_CJc = X_BM_is_identity
                                      ? RigidTransformd()
                                      : RigidTransformd(Vector3d(7, 8, 9));

    MultibodyPlant<double> plant(0.0);

    const RigidBody<double>& parent0 = plant.AddRigidBody("parent0");
    const RigidBody<double>& child0 = plant.AddRigidBody("child0");
    const PrismaticJoint<double>& joint0 = plant.AddJoint<PrismaticJoint>(
        "joint0", parent0, X_PJp, child0, X_CJc, Vector3d::UnitX());

    plant.Finalize();
    auto context = plant.CreateDefaultContext();

    const internal::Mobilizer<double>& mobilizer0 = joint0.GetMobilizerInUse();
    const RigidTransformd& X_PF =
        mobilizer0.inboard_frame().EvalPoseInBodyFrame(*context);
    const RigidTransformd& X_BM =
        mobilizer0.outboard_frame().EvalPoseInBodyFrame(*context);

    EXPECT_EQ(X_PF.IsExactlyIdentity(), X_PF_is_identity);
    EXPECT_EQ(X_BM.IsExactlyIdentity(), X_BM_is_identity);
    const RigidTransformd X_WP = RigidTransformd(Vector3d(1, 2, 3));
    plant.SetFreeBodyPose(&*context, parent0, X_WP);
    const RigidTransformd X_JpJc(Vector3d(100, 0, 0));
    joint0.set_translation(&*context, X_JpJc.translation().x());  // sets X_JpJc
    const RigidTransformd& X_WC = child0.EvalPoseInWorld(*context);

    EXPECT_EQ(X_WC.translation(), X_WP.translation() + X_PJp.translation() +
                                      X_JpJc.translation() +
                                      X_CJc.inverse().translation());
  };

  test_case(0);
  test_case(1);
  test_case(2);
  test_case(3);
}

// Verifies exact set of output ports we expect to be a direct feedthrough of
// the inputs. Returns true iff successful.
bool VerifyFeedthroughPorts(const MultibodyPlant<double>& plant) {
  // Write down the expected direct-feedthrough status for all ports. Use
  // strings (not indices) so that developers can understand test failures.
  // The order we list the ports here matches the MbP header file overview.
  // Nothing in the list below should ever be unconditionally `true`.
  const bool is_sampled = plant.has_sampled_output_ports();
  const std::vector<std::pair<std::string, bool>> manifest{
      {"state", false},
      {"body_poses", false},
      {"body_spatial_velocities", false},
      {"body_spatial_accelerations", !is_sampled},
      {"generalized_acceleration", !is_sampled},
      {"net_actuation", !is_sampled},
      {"reaction_forces", !is_sampled},
      {"contact_results", !is_sampled},
      // Grey group.
      {"{instance}_state", false},
      {"{instance}_generalized_acceleration", !is_sampled},
      {"{instance}_net_actuation", !is_sampled},
      {"{instance}_generalized_contact_forces", !is_sampled},
      // Green group.
      {"geometry_pose", false},
      {"deformable_body_configuration", false},
  };

  // Split the manifest into (non-)feedthrough sets, while also substituting and
  // expanding the per-model-instance port names.
  std::set<std::string> expected_feedthrough;
  std::set<std::string> expected_non_feedthrough;
  for (const auto& [name, is_feedthrough] : manifest) {
    std::set<std::string>& destination =
        is_feedthrough ? expected_feedthrough : expected_non_feedthrough;
    const bool needs_fmt = name.at(0) == '{';
    if (!needs_fmt) {
      destination.insert(name);
    } else {
      for (ModelInstanceIndex i(0); i < plant.num_model_instances(); ++i) {
        const std::string& instance = plant.GetModelInstanceName(i);
        destination.insert(
            fmt::format(fmt::runtime(name), fmt::arg("instance", instance)));
      }
    }
  }

  // Cross-check that every output port was listed in the manifest.
  {
    std::set<std::string> manifest_all;
    manifest_all.insert(expected_feedthrough.begin(),
                        expected_feedthrough.end());
    manifest_all.insert(expected_non_feedthrough.begin(),
                        expected_non_feedthrough.end());
    std::set<std::string> plant_all;
    for (OutputPortIndex i{0}; i < plant.num_output_ports(); ++i) {
      const auto& port = plant.get_output_port(i, /*warn_deprecated =*/false);
      plant_all.insert(port.get_name());
    }
    EXPECT_THAT(manifest_all, testing::ContainerEq(plant_all));
  }

  // Extract actual feedthrough status of the plant.
  std::set<OutputPortIndex> actual_feedthrough_indices;
  for (const auto& [_, out] : plant.GetDirectFeedthroughs()) {
    actual_feedthrough_indices.insert(OutputPortIndex{out});
  }
  std::set<std::string> actual_feedthrough;
  for (const OutputPortIndex& i : actual_feedthrough_indices) {
    actual_feedthrough.insert(
        plant.get_output_port(i, /*warn_deprecated=*/false).get_name());
  }

  // Compare and return.
  EXPECT_THAT(actual_feedthrough, testing::ContainerEq(expected_feedthrough));
  return actual_feedthrough == expected_feedthrough;
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

  DiagramBuilder<double> builder;
  auto systems = AddMultibodyPlantSceneGraph(&builder, 0.0);
  MultibodyPlant<double>& plant = systems.plant;
  SceneGraph<double>& scene_graph = systems.scene_graph;

  // A half-space for the ground geometry.
  CoulombFriction<double> ground_friction(0.5, 0.3);
  GeometryId ground_id = plant.RegisterCollisionGeometry(
      plant.world_body(),
      // A half-space passing through the origin in the x-z plane.
      geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero()),
      geometry::HalfSpace(), "ground", ground_friction);

  // Add two spherical bodies.
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& sphere1 =
      plant.AddRigidBody("Sphere1", SpatialInertia<double>::MakeUnitary());
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
      plant.AddRigidBody("Sphere2", SpatialInertia<double>::MakeUnitary());
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
  EXPECT_TRUE(VerifyFeedthroughPorts(plant));

  EXPECT_EQ(plant.num_visual_geometries(), 0);
  EXPECT_EQ(plant.num_collision_geometries(), 3);
  EXPECT_TRUE(plant.geometry_source_is_registered());
  EXPECT_TRUE(plant.get_source_id());

  auto diagram = builder.Build();
  unique_ptr<Context<double>> diagram_context = diagram->CreateDefaultContext();
  Context<double>* plant_context =
      &plant.GetMyMutableContextFromRoot(diagram_context.get());

  // Test the API taking a RigidTransform.
  auto X_WS1 = RigidTransformd(Vector3d(-x_offset, radius, 0.0));

  // Place sphere 1 on top of the ground, with offset x = -x_offset.
  plant.SetFreeBodyPose(plant_context, sphere1, X_WS1);
  // Place sphere 2 on top of the ground, with offset x = x_offset.
  plant.SetFreeBodyPose(plant_context, sphere2,
                        RigidTransformd(Vector3d(x_offset, radius, 0.0)));

  unique_ptr<AbstractValue> poses_value =
      plant.get_geometry_pose_output_port().Allocate();
  DRAKE_EXPECT_NO_THROW(poses_value->get_value<FramePoseVector<double>>());
  const FramePoseVector<double>& pose_data =
      poses_value->get_value<FramePoseVector<double>>();

  // Compute the poses for each geometry in the model.
  plant.get_geometry_pose_output_port().Calc(*plant_context, poses_value.get());
  EXPECT_EQ(pose_data.size(), 2);  // Only two frames move.

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  for (BodyIndex body_index(1); body_index < plant.num_bodies(); ++body_index) {
    const FrameId frame_id = plant.GetBodyFrameIdOrThrow(body_index);
    const RigidTransform<double>& X_WB = pose_data.value(frame_id);
    const RigidTransform<double>& X_WB_expected =
        plant.EvalBodyPoseInWorld(*plant_context, plant.get_body(body_index));
    EXPECT_TRUE(CompareMatrices(X_WB.GetAsMatrix34(),
                                X_WB_expected.GetAsMatrix34(), kTolerance,
                                MatrixCompareType::relative));
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

  // This is a convenient place to cover EvalSceneGraphInspector with a unit
  // test and cross-check the context inspector with the model inspector.
  const auto& context_inspector = plant.EvalSceneGraphInspector(*plant_context);
  const geometry::ProximityProperties* context_ground_props =
      context_inspector.GetProximityProperties(ground_id);
  ASSERT_NE(context_ground_props, nullptr);
  EXPECT_TRUE(context_ground_props->GetProperty<CoulombFriction<double>>(
                  geometry::internal::kMaterialGroup,
                  geometry::internal::kFriction) == ground_friction);
}

// Verifies the process of visual geometry registration with a SceneGraph. We
// have multiple APIs, so we register multiple geometries, one with each
// distinct API invocation. We build a model with multiple spheres and a ground
// plane. The ground plane is located at y = 0 with normal in the y-axis
// direction.
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
  scene_graph.AddRenderer("dummy", std::move(temp_engine));
  MultibodyPlant<double> plant(0.0);
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  EXPECT_EQ(render_engine.num_registered(), 0);

  const std::string model("test_model");
  const ModelInstanceIndex model_instance = plant.AddModelInstance(model);

  // A half-space for the ground geometry. The ground shape defines no visual
  // properties at all. It should get both roles and have the ("label", "id")
  // perception property.
  const GeometryId ground_id = plant.RegisterVisualGeometry(
      plant.world_body(),
      // A half-space passing through the origin in the x-z plane.
      geometry::HalfSpace::MakePose(Vector3d::UnitY(), Vector3d::Zero()),
      geometry::HalfSpace(), "ground");
  EXPECT_EQ(render_engine.num_registered(), 1);

  // Add the spherical bodies.
  // We just need a non-zero spatial inertia.
  const auto M_BBo = SpatialInertia<double>::MakeUnitary();
  const geometry::Sphere sphere(radius);
  const RigidTransformd X_BG = RigidTransformd::Identity();

  // Sphere1 defines a diffuse color. It should get both roles. It should have
  // the ("phong", "diffuse") property in both roles and the ("label", "id")
  // perception property.
  const RigidBody<double>& sphere1 =
      plant.AddRigidBody("Sphere1", model_instance, M_BBo);
  const Vector4<double> sphere1_diffuse{0.9, 0.1, 0.1, 0.5};
  // Turns the diffuse color into illustration and perception properties.
  const GeometryId sphere1_id = plant.RegisterVisualGeometry(
      sphere1, X_BG, sphere, "visual1", sphere1_diffuse);
  EXPECT_EQ(render_engine.num_registered(), 2);

  // Sphere2 defines illustration properties. It should get both roles with the
  // perception properties including all illustration properties and with the
  // ("label", "id") perception property.
  const RigidBody<double>& sphere2 =
      plant.AddRigidBody("Sphere2", model_instance, M_BBo);
  IllustrationProperties sphere2_props;
  const Vector4<double> sphere2_diffuse{0.1, 0.9, 0.1, 0.5};
  sphere2_props.AddProperty("phong", "diffuse", sphere2_diffuse);
  sphere2_props.AddProperty("phong", "diffuse_map", "empty.png");
  // This property prevents MbP from attempting to copy properties by hand. It
  // must rely on GeometryProperties copying abilities instead.
  sphere2_props.AddProperty("test_only", "dummy", true);
  sphere2_props.AddProperty("renderer", "accepting",
                            std::set<std::string>{"not_dummy"});
  // Uses illustration properties for both illustration and perception
  // properties.
  const GeometryId sphere2_id = plant.RegisterVisualGeometry(
      sphere2, X_BG, sphere, "visual2", sphere2_props);
  // Sphere2's accepting renderer is *not* `render_engine`; it will not be
  // added to it.
  EXPECT_EQ(render_engine.num_registered(), 2);

  // Sphere3 defines *only* perception properties for a geometry instance. It
  // should only get the perception role, but should have the ("label", "id")
  // property added.
  const RigidBody<double>& sphere3 =
      plant.AddRigidBody("Sphere3", model_instance, M_BBo);
  auto sphere3_instance =
      std::make_unique<GeometryInstance>(X_BG, sphere, "visual3");
  PerceptionProperties perception_props;
  sphere3_instance->set_perception_properties(perception_props);
  const GeometryId sphere3_id =
      plant.RegisterVisualGeometry(sphere3, std::move(sphere3_instance));
  EXPECT_EQ(render_engine.num_registered(), 3);

  // Sphere4 defines perception properties with a ("label", "id") property. It
  // will not change when added.
  const RigidBody<double>& sphere4 =
      plant.AddRigidBody("Sphere4", model_instance, M_BBo);
  auto sphere4_instance =
      std::make_unique<GeometryInstance>(X_BG, sphere, "visual4");
  perception_props.AddProperty("label", "id", RenderLabel(1234));
  sphere4_instance->set_perception_properties(perception_props);
  const GeometryId sphere4_id =
      plant.RegisterVisualGeometry(sphere4, std::move(sphere4_instance));
  EXPECT_EQ(render_engine.num_registered(), 4);

  // Sphere5 provides a geometry instance with no properties assigned. It will
  // be registered with "defaulted" properties for both visual roles.
  const RigidBody<double>& sphere5 =
      plant.AddRigidBody("Sphere5", model_instance, M_BBo);
  auto sphere5_instance =
      std::make_unique<GeometryInstance>(X_BG, sphere, "visual5");
  const GeometryId sphere5_id =
      plant.RegisterVisualGeometry(sphere3, std::move(sphere5_instance));
  EXPECT_EQ(render_engine.num_registered(), 5);

  // We are done defining the model.
  plant.Finalize();

  EXPECT_EQ(plant.num_visual_geometries(), 6);
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

  // This implicitly assumes that there *is* a ("phong", "diffuse") material. An
  // exception will be thrown otherwise.
  auto get_diffuse_color = [&inspector](GeometryId id) -> Vector4<double> {
    const IllustrationProperties* material =
        inspector.GetIllustrationProperties(id);
    return material->GetProperty<Vector4<double>>("phong", "diffuse");
  };

  // Get the render label property from the geometry's perception properties.
  auto get_render_label = [&inspector](GeometryId id) {
    const PerceptionProperties* material =
        inspector.GetPerceptionProperties(id);
    return material->GetProperty<RenderLabel>("label", "id");
  };

  // Confirms that every illustration property is also in the perception
  // properties.
  auto illustration_in_perception = [&inspector](GeometryId id) {
    const IllustrationProperties* illus =
        inspector.GetIllustrationProperties(id);
    if (illus == nullptr) return true;

    const PerceptionProperties* percep = inspector.GetPerceptionProperties(id);
    if (percep == nullptr) return false;

    for (const auto& group_name : illus->GetGroupNames()) {
      if (!percep->HasGroup(group_name)) return false;
      for (const auto& [name, val] : illus->GetPropertiesInGroup(group_name)) {
        // We're not going to worry about values. We'll assume if all of the
        // properties were copied, the values were well. We just need evidence
        // that copying took place.
        if (!percep->HasProperty(group_name, name)) return false;
      }
    }
    return true;
  };

  {
    const IllustrationProperties* material =
        inspector.GetIllustrationProperties(ground_id);
    ASSERT_NE(material, nullptr);
    // Undefined property value indicates use of default value.
    EXPECT_FALSE(material->HasProperty("phong", "diffuse"));
    EXPECT_EQ(get_render_label(ground_id),
              RenderLabel(plant.world_body().index()));
    EXPECT_TRUE(illustration_in_perception(ground_id));
    // Ground belongs to default model instance - no name scoping.
    EXPECT_EQ(inspector.GetName(ground_id), "ground");
  }

  {
    const Vector4<double>& test_diffuse = get_diffuse_color(sphere1_id);
    EXPECT_TRUE(CompareMatrices(test_diffuse, sphere1_diffuse, 0.0,
                                MatrixCompareType::absolute));
    EXPECT_EQ(get_render_label(sphere1_id), RenderLabel(sphere1.index()));
    EXPECT_TRUE(illustration_in_perception(sphere1_id));
    EXPECT_EQ(inspector.GetName(sphere1_id), "test_model::visual1");
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
    EXPECT_EQ(get_render_label(sphere2_id), RenderLabel(sphere2.index()));
    EXPECT_TRUE(illustration_in_perception(sphere2_id));
    EXPECT_EQ(inspector.GetName(sphere2_id), "test_model::visual2");
  }

  {
    EXPECT_EQ(inspector.GetIllustrationProperties(sphere3_id), nullptr);
    EXPECT_NE(inspector.GetPerceptionProperties(sphere3_id), nullptr);
    EXPECT_EQ(get_render_label(sphere3_id), RenderLabel(sphere3.index()));
    EXPECT_EQ(inspector.GetName(sphere3_id), "test_model::visual3");
  }

  {
    EXPECT_EQ(inspector.GetIllustrationProperties(sphere4_id), nullptr);
    EXPECT_NE(inspector.GetPerceptionProperties(sphere4_id), nullptr);
    EXPECT_NE(get_render_label(sphere4_id), RenderLabel(sphere4.index()));
    EXPECT_EQ(get_render_label(sphere4_id), RenderLabel(1234));
    EXPECT_EQ(inspector.GetName(sphere4_id), "test_model::visual4");
  }

  {
    EXPECT_NE(inspector.GetIllustrationProperties(sphere5_id), nullptr);
    EXPECT_NE(inspector.GetPerceptionProperties(sphere5_id), nullptr);
    EXPECT_NE(get_render_label(sphere5_id), RenderLabel(sphere5.index()));
    EXPECT_EQ(inspector.GetName(sphere5_id), "test_model::visual5");
    EXPECT_TRUE(illustration_in_perception(sphere5_id));
  }
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

  unique_ptr<LinearSystem<double>> linearized_pendulum = Linearize(
      *pendulum, *context, pendulum->get_actuation_input_port().get_index(),
      systems::OutputPortSelection::kNoOutput);

  // Compute the expected solution by hand.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  const double domegadot_domega =
      -parameters.damping() /
      (parameters.m() * parameters.l() * parameters.l());
  // clang-format off
  A << 0.0, 1.0,
       parameters.g() / parameters.l(), domegadot_domega;
  // clang-format on
  B << 0, 1 / (parameters.m() * parameters.l() * parameters.l());
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->A(), A, kTolerance));
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->B(), B, kTolerance));

  // Now we linearize about the stable fixed point with the pendulum in its
  // downward position.
  pin.set_angle(context.get(), 0.0);
  pin.set_angular_rate(context.get(), 0.0);
  linearized_pendulum = Linearize(
      *pendulum, *context, pendulum->get_actuation_input_port().get_index(),
      systems::OutputPortSelection::kNoOutput);
  // Compute the expected solution by hand.
  // clang-format off
  A << 0.0, 1.0,
       -parameters.g() / parameters.l(), domegadot_domega;
  // clang-format on
  B << 0, 1 / (parameters.m() * parameters.l() * parameters.l());
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
  // This is used in purely kinematic tests.
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& body =
      plant->AddRigidBody("FreeBody", SpatialInertia<double>::MakeUnitary());
  plant->Finalize();

  *context = plant->CreateDefaultContext();

  // Set an arbitrary pose of the body in the world.
  const Vector3d p_WB(1, 2, 3);  // Position in world.
  const Vector3d axis_W =        // Orientation in world.
      (1.5 * Vector3d::UnitX() + 2.0 * Vector3d::UnitY() +
       3.0 * Vector3d::UnitZ())
          .normalized();
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

  EXPECT_TRUE(plant.IsVelocityEqualToQDot());

  // Make sure that the mapping functions do not throw.
  BasicVector<double> qdot(0), v(0);
  ASSERT_NO_THROW(plant.MapVelocityToQDot(*context, v, &qdot));
  ASSERT_NO_THROW(plant.MapQDotToVelocity(*context, qdot, &v));
  ASSERT_NO_THROW(plant.MakeVelocityToQDotMap(*context));
  ASSERT_NO_THROW(plant.MakeQDotToVelocityMap(*context));
}

GTEST_TEST(MultibodyPlantTest, MapVelocityToQDotAndBackContinuous) {
  MultibodyPlant<double> plant(0.0);
  unique_ptr<Context<double>> context;
  InitializePlantAndContextForVelocityToQDotMapping(&plant, &context);

  EXPECT_FALSE(plant.IsVelocityEqualToQDot());

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

  Eigen::SparseMatrix<double> N = plant.MakeVelocityToQDotMap(*context);
  EXPECT_TRUE(CompareMatrices(qdot.value(), N * v.value(), kTolerance));
  Eigen::SparseMatrix<double> Nplus = plant.MakeQDotToVelocityMap(*context);
  EXPECT_TRUE(CompareMatrices(v.value(), Nplus * qdot.value(), kTolerance));
}

GTEST_TEST(MultibodyPlantTest, MapVelocityToQDotAndBackDiscrete) {
  const double time_step = 1e-3;
  MultibodyPlant<double> plant(time_step);
  unique_ptr<Context<double>> context;
  InitializePlantAndContextForVelocityToQDotMapping(&plant, &context);

  EXPECT_FALSE(plant.IsVelocityEqualToQDot());

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
    Parser(&plant_).AddModelsFromUrl(
        "package://drake/multibody/plant/test/split_pendulum.sdf");
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
  pin_->set_angle(context_.get(), theta);

  MatrixX<double> M_via_id(1, 1), M_via_W(1, 1);
  plant_.CalcMassMatrixViaInverseDynamics(*context_, &M_via_id);
  plant_.CalcMassMatrix(*context_, &M_via_W);

  // We can only expect values within the precision specified in the sdf file.
  EXPECT_NEAR(M_via_id(0, 0), Io, 1.0e-6);
  EXPECT_NEAR(M_via_W(0, 0), Io, 1.0e-6);
}

// This test ensures that we can create a symbolic mass matrix successfully.
TEST_F(SplitPendulum, SymbolicMassMatrix) {
  auto sym_plant = systems::System<double>::ToSymbolic(plant_);
  auto sym_context = sym_plant->CreateDefaultContext();

  // State variables
  const Eigen::VectorX<symbolic::Variable> q_var =
      symbolic::MakeVectorVariable(1, "q");
  const Eigen::VectorX<symbolic::Variable> v_var =
      symbolic::MakeVectorVariable(1, "v");

  // Parameters
  const Eigen::VectorX<symbolic::Variable> m_var =
      symbolic::MakeVectorVariable(2, "m");
  const Eigen::VectorX<symbolic::Variable> l_var =
      symbolic::MakeVectorVariable(2, "l");

  const Eigen::VectorX<symbolic::Expression> q = q_var, v = v_var, m = m_var,
                                             l = l_var;

  const auto& upper_arm = sym_plant->GetBodyByName("upper_section");
  const SpatialInertia<symbolic::Expression> inertia0(
      m[0], Vector3<symbolic::Expression>(0, 0, -l[0]),
      UnitInertia<symbolic::Expression>(l[0] * l[0], l[0] * l[0], 0));
  upper_arm.SetSpatialInertiaInBodyFrame(sym_context.get(), inertia0);
  const auto& lower_arm = sym_plant->GetBodyByName("lower_section");
  const SpatialInertia<symbolic::Expression> inertia1(
      m[1], Vector3<symbolic::Expression>(0, 0, -l[1]),
      UnitInertia<symbolic::Expression>(l[1] * l[1], l[1] * l[1], 0));
  lower_arm.SetSpatialInertiaInBodyFrame(sym_context.get(), inertia1);

  sym_plant->SetPositions(sym_context.get(), q);
  sym_plant->SetVelocities(sym_context.get(), v);

  // Calculate the mass matrix two different ways, via Inverse Dynamics
  // ("_via_id") and using the Composite Body Algorithm via recursion of
  // World-frame quantities ("_via_W"). Verify that evaluating the resulting
  // expressions yields the same result numerically.
  Eigen::MatrixX<symbolic::Expression> M_via_id(1, 1), M_via_W(1, 1);
  sym_plant->CalcMassMatrixViaInverseDynamics(*sym_context, &M_via_id);
  sym_plant->CalcMassMatrix(*sym_context, &M_via_W);

  const symbolic::Environment env{{q_var(0), 2.0}, {v_var(0), 10.},
                                  {m_var(0), 3.0}, {m_var(1), 4.0},
                                  {l_var(0), 5.0}, {l_var(1), 6.0}};
  EXPECT_NEAR(M_via_W(0, 0).Evaluate(env), M_via_id(0, 0).Evaluate(env), 1e-14);

  // Generate symbolic expressions for a few more quantities here just as a
  // sanity check that we can do so. We won't look at the results.
  Eigen::VectorX<symbolic::Expression> Cv(1), tauExt(1);
  sym_plant->CalcBiasTerm(*sym_context, &Cv);
  EXPECT_NO_THROW(sym_plant->CalcGravityGeneralizedForces(*sym_context));
  const Eigen::MatrixX<symbolic::Expression> B =
      sym_plant->MakeActuationMatrix();
  EXPECT_EQ(B.rows(), 1);
  EXPECT_EQ(B.cols(), 1);
  MultibodyForces<symbolic::Expression> forces(*sym_plant);
  sym_plant->CalcForceElementsContribution(*sym_context, &forces);
  sym_plant->CalcGeneralizedForces(*sym_context, forces, &tauExt);
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
      rigid_body =
          &mutable_tree().AddRigidBody("Body", SpatialInertia<double>::NaN());
      Finalize();
    }
    const RigidBody<double>* rigid_body{};
  } mb_system;

  DRAKE_EXPECT_THROWS_MESSAGE(
      mb_system.rigid_body->GetParentPlant(),
      ".*multibody element.*not owned by.*MultibodyPlant.*");
}

// Verifies we can parse link collision geometries and surface friction.
GTEST_TEST(MultibodyPlantTest, ScalarConversionConstructor) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  Parser(&plant, &scene_graph)
      .AddModelsFromUrl(
          "package://drake/multibody/parsing/test/"
          "links_with_visuals_and_collisions.sdf");

  // Try scalar-converting pre-finalize - error.
  DRAKE_EXPECT_THROWS_MESSAGE(systems::System<double>::ToAutoDiffXd(plant),
                              ".*MultibodyTree that has not been finalized.*");

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
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(plant);
  EXPECT_TRUE(plant_autodiff->geometry_source_is_registered());
  EXPECT_EQ(plant_autodiff->num_collision_geometries(),
            plant.num_collision_geometries());
  EXPECT_EQ(plant_autodiff
                ->GetCollisionGeometriesForBody(
                    plant_autodiff->GetBodyByName("link1"))
                .size(),
            link1_num_collisions);
  EXPECT_EQ(plant_autodiff
                ->GetCollisionGeometriesForBody(
                    plant_autodiff->GetBodyByName("link2"))
                .size(),
            link2_num_collisions);
  EXPECT_EQ(plant_autodiff
                ->GetCollisionGeometriesForBody(
                    plant_autodiff->GetBodyByName("link3"))
                .size(),
            link3_num_collisions);
  EXPECT_EQ(
      plant_autodiff
          ->GetVisualGeometriesForBody(plant_autodiff->GetBodyByName("link1"))
          .size(),
      link1_num_visuals);
  EXPECT_EQ(
      plant_autodiff
          ->GetVisualGeometriesForBody(plant_autodiff->GetBodyByName("link2"))
          .size(),
      link2_num_visuals);
  EXPECT_EQ(
      plant_autodiff
          ->GetVisualGeometriesForBody(plant_autodiff->GetBodyByName("link3"))
          .size(),
      link3_num_visuals);

  // Make sure the geometry ports were included in the autodiffed plant.
  DRAKE_EXPECT_NO_THROW(plant_autodiff->get_geometry_query_input_port());
  DRAKE_EXPECT_NO_THROW(plant_autodiff->get_geometry_pose_output_port());
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
    // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
    const RigidBody<double>& large_box =
        plant_.AddRigidBody("LargeBox", SpatialInertia<double>::MakeUnitary());
    large_box_id_ = plant_.RegisterCollisionGeometry(
        large_box, RigidTransformd::Identity(),
        geometry::Box(large_box_size_, large_box_size_, large_box_size_),
        "collision", CoulombFriction<double>());

    const RigidBody<double>& small_box =
        plant_.AddRigidBody("SmallBox", SpatialInertia<double>::MakeUnitary());
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

    const RigidBody<double>& large_box = plant_.GetBodyByName("LargeBox");
    const RigidBody<double>& small_box = plant_.GetBodyByName("SmallBox");

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
        RigidTransform<double>(
            RotationMatrix<double>::Identity(),
            Vector3<double>(0, small_box_size_ / 2.0 - penetration_, 0));

    plant_.SetFreeBodyPose(context, large_box, X_WLb);
    plant_.SetFreeBodyPose(context, small_box, X_WSb);
  }

  // Generate a valid set of penetrations for this particular setup that
  // emulates a multicontact scenario.
  void SetPenetrationPairs(
      const Context<double>& context,
      std::vector<PenetrationAsPointPair<double>>* penetrations) {
    const RigidBody<double>& large_box = plant_.GetBodyByName("LargeBox");
    const RigidBody<double>& small_box = plant_.GetBodyByName("SmallBox");

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
  pair<unique_ptr<MultibodyPlant<AutoDiffXd>>, unique_ptr<Context<AutoDiffXd>>>
  ConvertPlantAndContextToAutoDiffXd() {
    // Scalar convert the plant and its context_.
    unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
        systems::System<double>::ToAutoDiffXd(plant_);
    unique_ptr<Context<AutoDiffXd>> context_autodiff =
        plant_autodiff->CreateDefaultContext();
    context_autodiff->SetTimeStateAndParametersFrom(*context_);

    // Initialize v_autodiff to have values v and so that it is the independent
    // variable of the problem.
    const VectorX<double> v = context_->get_continuous_state()
                                  .get_generalized_velocity()
                                  .CopyToVector();
    VectorX<AutoDiffXd> v_autodiff(plant_.num_velocities());
    math::InitializeAutoDiff(v, &v_autodiff);
    context_autodiff->get_mutable_continuous_state()
        .get_mutable_generalized_velocity()
        .SetFromVector(v_autodiff);

    return make_pair(std::move(plant_autodiff), std::move(context_autodiff));
  }

  // Helper method to compute the separation velocity in the direction defined
  // by the normal nhat_BA for each contact pair in pairs_set. The i-th entry in
  // the output vector contains the separation velocity for the i-th pair in
  // pairs_set.
  // This method is templated to facilitate automatic differentiation for this
  // test.
  template <typename T>
  VectorX<T> CalcNormalVelocities(
      const MultibodyPlant<T>& plant_on_T, const Context<T>& context_on_T,
      const std::vector<PenetrationAsPointPair<double>>& pairs_set) const {
    VectorX<T> vn(pairs_set.size());
    int icontact = 0;
    for (const auto& pair : pairs_set) {
      PenetrationAsPointPair<T> pair_on_T;

      BodyIndex bodyA_index =
          MultibodyPlantTester::FindBodyByGeometryId(plant_on_T, pair.id_A);
      const RigidTransform<T>& X_WA = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyA_index));
      const SpatialVelocity<T> V_WA = plant_on_T.EvalBodySpatialVelocityInWorld(
          context_on_T, plant_on_T.get_body(bodyA_index));

      BodyIndex bodyB_index =
          MultibodyPlantTester::FindBodyByGeometryId(plant_on_T, pair.id_B);
      const RigidTransform<T>& X_WB = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyB_index));
      const SpatialVelocity<T> V_WB = plant_on_T.EvalBodySpatialVelocityInWorld(
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
      const MultibodyPlant<T>& plant_on_T, const Context<T>& context_on_T,
      const std::vector<PenetrationAsPointPair<double>>& pairs_set,
      const std::vector<RotationMatrix<double>>& R_WC_set) const {
    VectorX<T> vt(2 * pairs_set.size());
    int icontact = 0;
    for (const auto& pair : pairs_set) {
      PenetrationAsPointPair<T> pair_on_T;

      BodyIndex bodyA_index =
          MultibodyPlantTester::FindBodyByGeometryId(plant_on_T, pair.id_A);
      const RigidTransform<T>& X_WA = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyA_index));
      const SpatialVelocity<T> V_WA = plant_on_T.EvalBodySpatialVelocityInWorld(
          context_on_T, plant_on_T.get_body(bodyA_index));

      BodyIndex bodyB_index =
          MultibodyPlantTester::FindBodyByGeometryId(plant_on_T, pair.id_B);
      const RigidTransform<T>& X_WB = plant_on_T.EvalBodyPoseInWorld(
          context_on_T, plant_on_T.get_body(bodyB_index));
      const SpatialVelocity<T> V_WB = plant_on_T.EvalBodySpatialVelocityInWorld(
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
      vt(2 * icontact) = that1_W.dot(v_WCb - v_WCa);
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

// Verifies that we can obtain the indexes into the state vector for each joint
// in the model of a Kuka arm.
// For this topologically simple model with only one branch of bodies with root
// in the world, joints, and their degrees of freedom, are numbered from root
// (world) in increasing order towards the end effector.
GTEST_TEST(KukaModel, JointIndexes) {
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  const auto& base_link_frame = plant.GetFrameByName("iiwa_link_0");
  const Joint<double>& weld =
      plant.WeldFrames(plant.world_frame(), base_link_frame);
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 7);
  EXPECT_EQ(plant.num_velocities(), 7);

  // We expect the last joint to be the one WeldJoint fixing the model to the
  // world, since we added it last above with the call to WeldFrames().
  // We verify this assumption.
  ASSERT_EQ(weld.index(), plant.GetJointIndices().back());

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

  // We know all joints in our model, besides the last joint welding the
  // model to the world, are revolute joints with the name `iiwa_joint_X`,
  // where X = [1, 7]. MultibodyPlant gives no guarantees about correlation of
  // joint indices and state indices. The mapping can be obtained via
  // MultibodyPlant::StateSelectorMatrix(). However, for this simple model we
  // know that state indices will be assigned in the same order as joint
  // indices.
  std::vector<int> q_starts;
  std::vector<int> v_starts;
  for (JointIndex joint_index : plant.GetJointIndices()) {
    // Skip "weld" joint.
    if (joint_index == weld.index()) continue;
    const Joint<double>& joint = plant.get_joint(joint_index);

    const int expected_num_v = 1;
    const int expected_num_q = 1;
    EXPECT_EQ(joint.num_positions(), expected_num_q);
    EXPECT_EQ(joint.num_velocities(), expected_num_v);
    EXPECT_EQ(joint.position_start(), joint.velocity_start());
    q_starts.push_back(joint.position_start());
    v_starts.push_back(joint.velocity_start());

    // Confirm that the mutable accessor returns the same object.
    Joint<double>& mutable_joint = plant.get_mutable_joint(joint_index);
    EXPECT_EQ(&mutable_joint, &joint);
  }
  EXPECT_THAT(q_starts, testing::ElementsAre(0, 1, 2, 3, 4, 5, 6));
  EXPECT_THAT(v_starts, testing::ElementsAre(0, 1, 2, 3, 4, 5, 6));

  // Verify that the indexes above point to the right entries in the state
  // stored in the context.
  auto context = plant.CreateDefaultContext();

  for (int joint_number : {1, 2, 3, 4, 5, 6, 7}) {
    const auto& joint = plant.GetJointByName<RevoluteJoint>(
        "iiwa_joint_" + std::to_string(joint_number));

    // We simply set each entry in the state with the value of its index.
    joint.set_angle(context.get(), joint.position_start());
    joint.set_angular_rate(context.get(),
                           plant.num_positions() + joint.velocity_start());
  }

  // Verify that each entry has the value we expect it to have.
  const VectorX<double> xc =
      context->get_continuous_state_vector().CopyToVector();
  const VectorX<double> xc_expected = VectorX<double>::LinSpaced(
      plant.num_multibody_states() /* size */, 0 /* first index */,
      plant.num_multibody_states() - 1 /* last index */);

  EXPECT_EQ(xc, xc_expected);
}

GTEST_TEST(KukaModel, ActuationMatrix) {
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"));
  plant.Finalize();

  EXPECT_EQ(plant.num_positions(), 7);
  EXPECT_EQ(plant.num_velocities(), 7);
  EXPECT_EQ(plant.num_actuated_dofs(), 7);

  const Eigen::MatrixXd B = plant.MakeActuationMatrix();
  EXPECT_EQ(B.rows(), plant.num_velocities());
  EXPECT_EQ(B.cols(), plant.num_actuated_dofs());
  const Eigen::SparseMatrix<double> B_inv =
      plant.MakeActuationMatrixPseudoinverse();
  EXPECT_EQ(B_inv.rows(), plant.num_actuated_dofs());
  EXPECT_EQ(B_inv.cols(), plant.num_velocities());
  EXPECT_TRUE((B * B_inv).isIdentity());
  EXPECT_TRUE((B_inv * B).isIdentity());
}

TEST_F(MultibodyPlantRemodelingDiscrete, MakeActuationMatrix) {
  BuildModel();
  DoRemoval(true /* remove actuator */, false /* do not remove joint */);
  FinalizeAndBuild();

  // We didn't remove any bodies, all indexes from 0 to num_bodies() should be
  // present.
  for (BodyIndex b(0); b < plant_->num_bodies(); ++b) {
    EXPECT_TRUE(plant_->has_body(b));
  }

  // Actuator with index 1 has been removed.
  // clang-format off
  const Eigen::MatrixXd B_expected =
        (Eigen::MatrixXd(3, 2) << 1, 0,
                                  0, 0,
                                  0, 1).finished();
  const Eigen::MatrixXd B_inv_expected =
        (Eigen::MatrixXd(2, 3) << 1, 0, 0,
                                  0, 0, 1).finished();
  // clang-format on

  // Test that MakeActuationMatrix uses the correct indices into 'u'
  // using JointActuator::input_start().
  const Eigen::MatrixXd B = plant_->MakeActuationMatrix();
  EXPECT_TRUE(CompareMatrices(B, B_expected));
  const Eigen::MatrixXd B_inv = plant_->MakeActuationMatrixPseudoinverse();
  EXPECT_TRUE(CompareMatrices(B_inv, B_inv_expected));
}

TEST_F(MultibodyPlantRemodelingDiscrete, MakeActuatorSelectorMatrix) {
  BuildModel();
  DoRemoval(true /* remove actuator */, false /* do not remove joint */);
  FinalizeAndBuild();

  // Actuator with index 1 ("actuator1") has been removed.
  // Flip the order of the actuators in the user list compared to the input
  // ordering.
  const std::vector<JointActuatorIndex> user_to_actuator_index_map{
      plant_->GetJointActuatorByName("actuator2").index(),
      plant_->GetJointActuatorByName("actuator0").index()};

  // clang-format off
  const Eigen::MatrixXd Su_expected =
        (Eigen::MatrixXd(2, 2) << 0, 1,
                                  1, 0).finished();
  // clang-format on

  // Test that MakeActuationSelectorMatrix uses the correct indices into 'u'
  // using JointActuator::input_start().
  const Eigen::MatrixXd Su =
      plant_->MakeActuatorSelectorMatrix(user_to_actuator_index_map);
  EXPECT_TRUE(CompareMatrices(Su, Su_expected));
}

TEST_F(MultibodyPlantRemodelingDiscrete, AddJointActuationForces) {
  BuildModel();
  DoRemoval(true /* remove actuator */, false /* do not remove joint */);
  FinalizeAndBuild();

  // Actuator with index 1 has been removed.
  const systems::InputPort<double>& u_input =
      plant_->get_actuation_input_port();
  u_input.FixValue(plant_context_, Vector2d(0.25, 0.5));

  const VectorXd forces_expected = Vector3d(0.25, 0.0, 0.5);

  // Test that AddJointActuationForces uses the correct indices into 'u'
  // using JointActuator::input_start().
  VectorXd forces(3);
  forces.setZero();
  MultibodyPlantTester::AddJointActuationForces(*plant_, *plant_context_,
                                                &forces);
  EXPECT_TRUE(CompareMatrices(forces, forces_expected));
}

TEST_F(MultibodyPlantRemodelingDiscrete, RemoveJoint) {
  BuildModel();
  DoRemoval(true /* remove actuator */, true /* remove joint */);
  // Before finalize we remove `joint1`. This makes body1 a free
  // body, but it will not have a joint until after finalize.
  EXPECT_EQ(plant_->num_joints(), 2);
  EXPECT_TRUE(plant_->has_joint(JointIndex(0)));
  EXPECT_FALSE(plant_->has_joint(JointIndex(1)));
  EXPECT_TRUE(plant_->has_joint(JointIndex(2)));
  EXPECT_TRUE(plant_->HasJointNamed("joint0"));
  EXPECT_FALSE(plant_->HasJointNamed("joint1"));
  EXPECT_TRUE(plant_->HasJointNamed("joint2"));
  EXPECT_THAT(plant_->GetJointIndices(),
              testing::ElementsAre(JointIndex(0), JointIndex(2)));

  // Validate that ordinals are assigned and updated contiguously.
  const Joint<double>& joint0 = plant_->get_joint(JointIndex(0));
  const Joint<double>& joint2 = plant_->get_joint(JointIndex(2));
  EXPECT_EQ(joint0.ordinal(), JointOrdinal(0));
  EXPECT_EQ(joint2.ordinal(), JointOrdinal(1));

  FinalizeAndBuild();

  // After finalize, `body1` is given a 6-dof joint.
  // The newly added joint should get the next available joint index.
  EXPECT_EQ(plant_->num_joints(), 3);
  EXPECT_TRUE(plant_->has_joint(JointIndex(0)));
  EXPECT_FALSE(plant_->has_joint(JointIndex(1)));
  EXPECT_TRUE(plant_->has_joint(JointIndex(2)));
  EXPECT_TRUE(plant_->has_joint(JointIndex(3)));
  EXPECT_TRUE(plant_->HasJointNamed("joint0"));
  EXPECT_FALSE(plant_->HasJointNamed("joint1"));
  EXPECT_TRUE(plant_->HasJointNamed("joint2"));
  EXPECT_TRUE(plant_->HasJointNamed("body1"));
  EXPECT_THAT(
      plant_->GetJointIndices(),
      testing::ElementsAre(JointIndex(0), JointIndex(2), JointIndex(3)));

  const QuaternionFloatingJoint<double>& body1_floating_joint =
      plant_->GetJointByName<QuaternionFloatingJoint>("body1");
  EXPECT_EQ(body1_floating_joint.index(), JointIndex(3));

  EXPECT_EQ(joint0.ordinal(), JointOrdinal(0));
  EXPECT_EQ(joint2.ordinal(), JointOrdinal(1));
  EXPECT_EQ(body1_floating_joint.ordinal(), JointOrdinal(2));

  // Confirm that removed joint logic is preserved after cloning.
  std::unique_ptr<MultibodyPlant<double>> clone =
      systems::System<double>::Clone(*plant_);
  EXPECT_EQ(clone->num_joints(), 3);
  EXPECT_TRUE(clone->has_joint(JointIndex(0)));
  EXPECT_FALSE(clone->has_joint(JointIndex(1)));
  EXPECT_TRUE(clone->has_joint(JointIndex(2)));
  EXPECT_TRUE(clone->has_joint(JointIndex(3)));
  EXPECT_TRUE(clone->HasJointNamed("joint0"));
  EXPECT_FALSE(clone->HasJointNamed("joint1"));
  EXPECT_TRUE(clone->HasJointNamed("joint2"));
  EXPECT_TRUE(clone->HasJointNamed("body1"));
  EXPECT_THAT(
      clone->GetJointIndices(),
      testing::ElementsAre(JointIndex(0), JointIndex(2), JointIndex(3)));
  EXPECT_EQ(clone->get_joint(JointIndex(0)).ordinal(), JointOrdinal(0));
  EXPECT_EQ(clone->get_joint(JointIndex(2)).ordinal(), JointOrdinal(1));
  EXPECT_EQ(clone->get_joint(JointIndex(3)).ordinal(), JointOrdinal(2));
}

TEST_F(MultibodyPlantRemodelingDiscrete, RemoveAndReplaceJoint) {
  BuildModel();
  DoRemoval(true /* remove actuator */, true /* remove joint */);
  constexpr int num_replacements = 100;
  // Exercise replacement of the joint multiple times. Each time will result
  // in a new joint index, it's ordinal should remain the same.
  for (int i = 1; i < num_replacements; ++i) {
    const RevoluteJoint<double>& joint1 = plant_->AddJoint<RevoluteJoint>(
        "joint1", plant_->GetBodyByName("body0"), {},
        plant_->GetBodyByName("body1"), {}, Vector3d::UnitZ());
    EXPECT_EQ(joint1.index(), JointIndex(2 + i));
    EXPECT_EQ(joint1.ordinal(), JointOrdinal(2));
    plant_->RemoveJoint(joint1);
  }

  // Replace the joint with a different joint type.
  const WeldJoint<double>& joint1 = plant_->AddJoint<WeldJoint>(
      "joint1", plant_->GetBodyByName("body0"), {},
      plant_->GetBodyByName("body1"), {}, RigidTransformd::Identity());
  EXPECT_EQ(joint1.index(), JointIndex(2 + num_replacements));

  // Check expected plant invariants.
  EXPECT_EQ(plant_->num_joints(), 3);
  EXPECT_TRUE(plant_->has_joint(JointIndex(0)));
  EXPECT_TRUE(plant_->has_joint(JointIndex(2)));
  EXPECT_TRUE(plant_->has_joint(JointIndex(2 + num_replacements)));
  EXPECT_TRUE(plant_->HasJointNamed("joint0"));
  EXPECT_TRUE(plant_->HasJointNamed("joint1"));
  EXPECT_TRUE(plant_->HasJointNamed("joint2"));
  EXPECT_THAT(plant_->GetJointIndices(),
              testing::ElementsAre(JointIndex(0), JointIndex(2),
                                   JointIndex(2 + num_replacements)));
  // Validate that ordinals are assigned and updated contiguously.
  const Joint<double>& joint0 = plant_->get_joint(JointIndex(0));
  const Joint<double>& joint2 = plant_->get_joint(JointIndex(2));
  EXPECT_EQ(joint0.ordinal(), JointOrdinal(0));
  EXPECT_EQ(joint2.ordinal(), JointOrdinal(1));
  EXPECT_EQ(joint1.ordinal(), JointOrdinal(2));

  FinalizeAndBuild();

  // Check the same invariants post finalize.
  EXPECT_EQ(plant_->num_joints(), 3);
  EXPECT_TRUE(plant_->has_joint(JointIndex(0)));
  EXPECT_TRUE(plant_->has_joint(JointIndex(2)));
  EXPECT_TRUE(plant_->has_joint(JointIndex(2 + num_replacements)));
  EXPECT_TRUE(plant_->HasJointNamed("joint0"));
  EXPECT_TRUE(plant_->HasJointNamed("joint1"));
  EXPECT_TRUE(plant_->HasJointNamed("joint2"));
  EXPECT_THAT(plant_->GetJointIndices(),
              testing::ElementsAre(JointIndex(0), JointIndex(2),
                                   JointIndex(2 + num_replacements)));
  EXPECT_EQ(joint0.ordinal(), JointOrdinal(0));
  EXPECT_EQ(joint2.ordinal(), JointOrdinal(1));
  EXPECT_EQ(joint1.ordinal(), JointOrdinal(2));
}

TEST_F(MultibodyPlantRemodelingDiscrete, RemoveJointWithActuator) {
  BuildModel();
  DRAKE_EXPECT_THROWS_MESSAGE(
      DoRemoval(false /* do not remove actuator */, true /* remove joint */),
      "RemoveJoint: joint with index.*has the following dependent model "
      "elements which must be removed prior to joint removal.*JointActuator.*");
}

TEST_P(MultibodyPlantRemodelingParam, RemoveJointWithCoupler) {
  BuildModel();
  // Add a coupler constraint between joint0 and joint1 before removal.
  plant_->AddCouplerConstraint(plant_->GetJointByName<RevoluteJoint>("joint0"),
                               plant_->GetJointByName<RevoluteJoint>("joint1"),
                               2.0 /* gear ratio */, 1.0 /* offset */);
  DRAKE_EXPECT_THROWS_MESSAGE(
      DoRemoval(true /* remove actuator */, true /* remove joint */),
      "RemoveJoint: This plant has 1 user-added constraints. This plant must "
      "have 0 user-added constraints in order to remove joint with index.*");
}

TEST_F(MultibodyPlantRemodelingDiscrete, RemoveJointWithDoorHinge) {
  BuildModel();
  // Add a DoorHinge with joint1 before removal.
  plant_->AddForceElement<DoorHinge>(
      plant_->GetJointByName<RevoluteJoint>("joint1"), DoorHingeConfig());
  DRAKE_EXPECT_THROWS_MESSAGE(
      DoRemoval(true /* remove actuator */, true /* remove joint */),
      "RemoveJoint: This plant has 1 user-added force elements. This plant "
      "must have 0 user-added force elements in order to remove joint with "
      "index.*");
}

TEST_F(MultibodyPlantRemodelingDiscrete, RemoveJointWithRevoluteSpring) {
  BuildModel();
  // Add a RevoluteSpring with joint1 before removal.
  plant_->AddForceElement<RevoluteSpring>(
      plant_->GetJointByName<RevoluteJoint>("joint1"), 0 /* nominal angle */,
      10 /* stiffness */);
  DRAKE_EXPECT_THROWS_MESSAGE(
      DoRemoval(true /* remove actuator */, true /* remove joint */),
      "RemoveJoint: This plant has 1 user-added force elements. This plant "
      "must have 0 user-added force elements in order to remove joint with "
      "index.*");
}

TEST_F(MultibodyPlantRemodelingDiscrete, RemoveJointWithPrismaticSpring) {
  BuildModel<PrismaticJoint>();
  // Add a PrismaticSpring with joint1 before removal.
  plant_->AddForceElement<PrismaticSpring>(
      plant_->GetJointByName<PrismaticJoint>("joint1"), 0 /* nominal angle */,
      10 /* stiffness */);
  DRAKE_EXPECT_THROWS_MESSAGE(
      DoRemoval(true /* remove actuator */, true /* remove joint */),
      "RemoveJoint: This plant has 1 user-added force elements. This plant "
      "must have 0 user-added force elements in order to remove joint with "
      "index.*");
}

TEST_F(MultibodyPlantRemodelingDiscrete, RemoveJointActuator) {
  BuildModel();

  for (bool has_actuator : {true, false}) {
    // The first time through the loop, the actuator remains intact.
    // The second time through, we'll remove it.
    if (!has_actuator) {
      DoRemoval(true /* remove_actuator */, false /* remove joint */);
    }
    // Check whether the joint exists or was removed.
    EXPECT_EQ(plant_->HasJointActuatorNamed("actuator1"), has_actuator);
    EXPECT_EQ(
        plant_->HasJointActuatorNamed("actuator1", default_model_instance()),
        has_actuator);
    EXPECT_EQ(plant_->has_joint_actuator(JointActuatorIndex{1}), has_actuator);
  }

  // This function only works post-finalize.
  plant_->Finalize();
  EXPECT_THAT(
      plant_->GetJointActuatorIndices(default_model_instance()),
      testing::ElementsAre(JointActuatorIndex{0}, JointActuatorIndex{2}));
}

// Unit test fixture for a model of Kuka Iiwa arm parametrized on the periodic
// update period of the plant. This allows us to test some of the plant's
// functionality for both continuous and discrete models.
class KukaArmTest : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() override {
    plant_ = std::make_unique<MultibodyPlant<double>>(this->GetParam());
    Parser(plant_.get())
        .AddModelsFromUrl(
            "package://drake_models/iiwa_description/sdf/"
            "iiwa14_no_collision.sdf");
    const Joint<double>& weld = plant_->WeldFrames(
        plant_->world_frame(), plant_->GetFrameByName("iiwa_link_0"));
    plant_->Finalize();

    // Only accelerations and joint reaction forces feedthrough, for either
    // continuous or discrete plants.
    EXPECT_TRUE(VerifyFeedthroughPorts(*plant_));

    EXPECT_EQ(plant_->num_positions(), 7);
    EXPECT_EQ(plant_->num_velocities(), 7);

    // We expect the last joint to be the one WeldJoint fixing the model to the
    // world, since we added it last above with the call to WeldFrames().
    // We verify this assumption.
    ASSERT_EQ(weld.index(), plant_->GetJointIndices().back());

    context_ = plant_->CreateDefaultContext();
  }

  // Helper to set the multibody state x to x[i] = xc[i] for each i-th entry in
  // the state vector. We use RevoluteJoint's methods to set the state in order
  // to independently unit test the proper workings of
  // MultibodyTree::get_multibody_state_vector() and its mutable counterpart.
  void SetState(const VectorX<double>& xc) {
    const int nq = plant_->num_positions();
    const JointIndex weld_index = plant_->GetJointIndices().back();
    for (JointIndex joint_index : plant_->GetJointIndices()) {
      // Skip "weld" joint.
      if (joint_index == weld_index) continue;

      const auto& joint = plant_->GetJointByName<RevoluteJoint>(
          plant_->get_joint(joint_index).name());

      // For this simple model we do know the order in which variables are
      // stored in the state vector.
      const double angle = xc[joint.ordinal()];
      const double angle_rate = xc[nq + joint.ordinal()];

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
      plant_->num_multibody_states() /* size */, 1 /* first index */,
      plant_->num_multibody_states() /* last index */);
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
  for (int i = 0; i < plant_->num_positions(); ++i) xc_expected[i] *= -1;
  plant_->SetPositions(context_.get(),
                       xc_expected.head(plant_->num_positions()));
  EXPECT_EQ(plant_->GetPositions(*context_),
            xc_expected.head(plant_->num_positions()));

  // Modify velocities and change xc_expected to reflect changes to velocities.
  for (int i = 0; i < plant_->num_velocities(); ++i)
    xc_expected[i + plant_->num_positions()] *= -1;
  plant_->SetVelocities(context_.get(),
                        xc_expected.tail(plant_->num_velocities()));
  EXPECT_EQ(plant_->GetVelocities(*context_),
            xc_expected.tail(plant_->num_velocities()));
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

TEST_P(KukaArmTest, EffortLimits) {
  Eigen::VectorXd effort_limits(7);
  effort_limits << 320, 320, 176, 176, 110, 40, 40;
  EXPECT_TRUE(CompareMatrices(plant_->GetEffortLowerLimits(), -effort_limits));
  EXPECT_TRUE(CompareMatrices(plant_->GetEffortUpperLimits(), effort_limits));

  plant_->RemoveAllJointActuatorEffortLimits();
  effort_limits.setConstant(std::numeric_limits<double>::infinity());
  EXPECT_TRUE(CompareMatrices(plant_->GetEffortLowerLimits(), -effort_limits));
  EXPECT_TRUE(CompareMatrices(plant_->GetEffortUpperLimits(), effort_limits));
}

TEST_P(KukaArmTest, InstanceStateAccess) {
  // Redo the setup process, now with two Iiwa's.
  const char kSdfUrl[] =
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
  plant_ = std::make_unique<MultibodyPlant<double>>(this->GetParam());
  Parser parser(plant_.get());
  multibody::ModelInstanceIndex arm1 =
      Parser(plant_.get(), "arm1").AddModelsFromUrl(kSdfUrl).at(0);
  multibody::ModelInstanceIndex arm2 =
      Parser(plant_.get(), "arm2").AddModelsFromUrl(kSdfUrl).at(0);
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
      plant_->num_positions(arm2) /* size */, 1 /* first number */,
      plant_->num_positions(arm2) /* last number */);
  VectorX<double> qd = VectorX<double>::LinSpaced(
      plant_->num_velocities(arm2) /* size */, 10 /* first number */,
      9 + plant_->num_velocities(arm2) /* last number */);
  VectorX<double> x(q.size() + qd.size());
  x << q, qd;

  // Set the positions, make sure that they're retrieved successfully, and
  // verify that no other multibody instance positions or velocities are
  // altered.
  plant_->SetPositionsAndVelocities(
      context_.get(), VectorXd::Zero(plant_->num_multibody_states()));
  plant_->SetPositions(context_.get(), arm2, q);
  EXPECT_EQ(plant_->GetPositions(*context_, arm2), q);
  EXPECT_EQ(plant_->GetPositions(*context_, arm1).norm(), 0);
  EXPECT_EQ(plant_->GetVelocities(*context_, arm1).norm(), 0);
  EXPECT_EQ(plant_->GetVelocities(*context_, arm2).norm(), 0);

  // Set the velocities, make sure that they're retrieved successfully, and
  // verify that no other multibody instance positions or velocities are
  // altered.
  plant_->SetPositionsAndVelocities(
      context_.get(), VectorXd::Zero(plant_->num_multibody_states()));
  plant_->SetVelocities(context_.get(), arm2, qd);
  EXPECT_EQ(plant_->GetVelocities(*context_, arm2), qd);
  EXPECT_EQ(plant_->GetPositions(*context_, arm1).norm(), 0);
  EXPECT_EQ(plant_->GetVelocities(*context_, arm1).norm(), 0);
  EXPECT_EQ(plant_->GetPositions(*context_, arm2).norm(), 0);

  // Set the positions and velocities, make sure that they're retrieved
  // successfully and verify that no other multibody instance positions or
  // velocities are altered.
  plant_->SetPositionsAndVelocities(
      context_.get(), VectorXd::Zero(plant_->num_multibody_states()));
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
    plant_->SetPositions(*context_, &context_->get_mutable_state(), arm2,
                         q_block);
    plant_->SetVelocities(context_.get(), arm2, v_block);
    plant_->SetVelocities(*context_, &context_->get_mutable_state(), arm2,
                          v_block);
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
      "Output array is not properly sized.");
  DRAKE_EXPECT_THROWS_MESSAGE(plant_->GetPositions(*context_, arm2, &qv_out),
                              "Output array is not properly sized.");
  DRAKE_EXPECT_THROWS_MESSAGE(plant_->GetVelocities(*context_, arm2, &qv_out),
                              "Output array is not properly sized.");

  // Test the GetPositionsFromArray and GetVelocitiesFromArray functionality.
  // Use qv_out as the state vector.
  VectorX<double> q_out_array(q_block.size());
  VectorX<double> v_out_array(v_block.size());
  VectorX<double> state_vector = plant_->GetPositionsAndVelocities(*context_);

  {
    // Ensure that getters accepting an output vector do not allocate heap.
    drake::test::LimitMalloc guard({.max_num_allocations = 0});
    plant_->GetPositionsFromArray(
        arm2, state_vector.head(plant_->num_positions()), &q_out_array);
    plant_->GetVelocitiesFromArray(
        arm2, state_vector.tail(plant_->num_velocities()), &v_out_array);
  }

  // Verify values.
  EXPECT_EQ(q_out_array, q_block);
  EXPECT_EQ(v_out_array, v_block);

  // Verify GetPositionsFromArray and GetVelocitiesFromArray error case.
  VectorX<double> q_out_array_err(q_block.size() + 1);
  VectorX<double> v_out_array_err(v_block.size() + 1);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->GetPositionsFromArray(
          arm2, state_vector.head(plant_->num_positions()), &q_out_array_err),
      "Output array is not properly sized.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->GetVelocitiesFromArray(
          arm2, state_vector.tail(plant_->num_velocities()), &v_out_array_err),
      "Output array is not properly sized.");
}

// Verifies we instantiated an appropriate MultibodyPlant model based on the
// fixture's parameter.
TEST_P(KukaArmTest, CheckContinuousOrDiscreteModel) {
  // The plant must not be a discrete system if the periodic update period is
  // zero.
  EXPECT_EQ(!plant_->is_discrete(), this->GetParam() == 0);
}

INSTANTIATE_TEST_SUITE_P(Blank, KukaArmTest,
                         testing::Values(0.0 /* continuous state */,
                                         1e-3 /* discrete state */));

GTEST_TEST(StateSelection, JointHasNoActuator) {
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelsFromUrl(
      "package://drake/multibody/benchmarks/acrobot/acrobot.sdf");
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
      "Joint 'ShoulderJoint' does not have an actuator.");
}

GTEST_TEST(StateSelection, KukaWithSimpleGripper) {
  const char kArmSdfUrl[] =
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
  const char kWsg50SdfUrl[] =
      "package://drake_models/wsg_50_description/sdf/schunk_wsg_50.sdf";

  // We make a "floating" model of a Kuka arm with a Schunk WSG 50 gripper at
  // the end effector. The purpose of having this floating model is to unit test
  // the methods for making state/actuation selector matrices for more complex
  // cases when we have floating robots in the model.
  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);
  const ModelInstanceIndex arm_model =
      parser.AddModelsFromUrl(kArmSdfUrl).at(0);

  // Add the gripper.
  const ModelInstanceIndex gripper_model =
      parser.AddModelsFromUrl(kWsg50SdfUrl).at(0);
  const auto& end_effector = plant.GetBodyByName("iiwa_link_7", arm_model);
  const auto& gripper_body = plant.GetBodyByName("body", gripper_model);
  // We don't care for the actual pose of the gripper in the end effector frame
  // for this example. We only care about the state size.
  plant.WeldFrames(end_effector.body_frame(), gripper_body.body_frame());
  plant.Finalize();

  // Assert the base of the robot is free and modeled with a quaternion before
  // moving on with this assumption.
  ASSERT_TRUE(plant.GetBodyByName("iiwa_link_0").is_floating_base_body());
  ASSERT_TRUE(plant.GetBodyByName("iiwa_link_0").has_quaternion_dofs());

  // Sanity check basic invariants.
  const int num_floating_positions = 7;
  const int num_floating_velocities = 6;

  // Seven dofs for the arm and two for the gripper, plus floating base.
  EXPECT_EQ(plant.num_positions(), 9 + num_floating_positions);
  EXPECT_EQ(plant.num_velocities(), 9 + num_floating_velocities);

  // Selected joints by name.
  const std::vector<std::string> arm_selected_joints_by_name = {
      "iiwa_joint_2", "iiwa_joint_7", "iiwa_joint_3"};

  std::vector<JointIndex> arm_selected_joints;
  // For this example we are only interested in the state for joints:
  //  - iiwa_joint_2
  //  - iiwa_joint_7
  //  - iiwa_joint_3
  // In that order.
  // We therefore create a user to joint index map accordingly.
  for (const auto& joint_name : arm_selected_joints_by_name) {
    arm_selected_joints.push_back(plant.GetJointByName(joint_name).index());
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
      OldMakeStateSelectorMatrixFromJointNames(arm_selected_joints_by_name);
  EXPECT_EQ(Sx_arm_by_name, Sx_arm_expected);

  // Intentionally attempt to create a state selector from a vector with
  // repeated joint names in order to verify the method throws.
  const std::vector<std::string> repeated_joint_names = {
      "iiwa_joint_2", "iiwa_joint_3", "iiwa_joint_7", "iiwa_joint_3"};
  DRAKE_EXPECT_THROWS_MESSAGE(
      OldMakeStateSelectorMatrixFromJointNames(repeated_joint_names),
      "Joint named 'iiwa_joint_3' is repeated multiple times.");

  // Intentionally attempt to create a state selector from a vector with
  // repeated joint indexes in order to verify the method throws.
  std::vector<JointIndex> repeated_joint_indexes;
  for (const auto& joint_name : repeated_joint_names) {
    repeated_joint_indexes.push_back(plant.GetJointByName(joint_name).index());
  }
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.MakeStateSelectorMatrix(repeated_joint_indexes),
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
      OldMakeStateSelectorMatrixFromJointNames(std::vector<std::string>());
  EXPECT_EQ(Sx_from_empty_names.rows(), 0);
  EXPECT_EQ(Sx_from_empty_names.cols(), plant.num_multibody_states());

  const MatrixX<double> Sx_from_empty_indexes =
      plant.MakeStateSelectorMatrix(std::vector<JointIndex>());
  EXPECT_EQ(Sx_from_empty_indexes.rows(), 0);
  EXPECT_EQ(Sx_from_empty_indexes.cols(), plant.num_multibody_states());

  const MatrixX<double> Su_from_empty_actuators =
      plant.MakeActuatorSelectorMatrix(std::vector<JointActuatorIndex>());
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
  // B is not invertible.
  const Eigen::SparseMatrix<double> Bplus =
      plant.MakeActuationMatrixPseudoinverse();
  EXPECT_EQ(Bplus.rows(), nu);
  EXPECT_EQ(Bplus.cols(), nv);
  // since nv >= nu, it's a true left inverse.
  EXPECT_TRUE((Bplus * B).isIdentity());

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
  unused(plant.MakeActuatorSelectorMatrix(std::vector<JointActuatorIndex>()));
}

// Confirms the free body APIs w.r.t. bodies that have a floating mobilizer
// but whose parent frame is not the world. They are still free and the
// various APIs should interpret poses as documented.
//
// We load a table and mug. The table is given a prismatic joint to the world
// (to facilitate testing spatial velocities), the mug is free. In some cases,
// it is a "floating base body" (its parent is the world) and sometimes only a
// "free body" (its parent is the table).
//
// This test explicitly omits some APIs because they are tested elsewhere:
//   - GTEST_TEST(StateSelection, FloatingBodies) tests
//     SetFloatingBaseBodyPoseInAnchoredFrame.
//   - GTEST_TEST(SetRandomTest, FloatingBodies) the random distribution of
//     of free body poses, as that is dealt with below in
//   - multibody_plant_introspection_test.cc covers the "UniqueFreeBody" APIs.
GTEST_TEST(StateSelection, FreeBodiesVsFloatingBaseBodies) {
  const std::string table_sdf_url =
      "package://drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf";
  const std::string mug_sdf_url =
      "package://drake/examples/simple_gripper/simple_mug.sdf";

  for (bool mug_in_world : {true, false}) {
    SCOPED_TRACE(mug_in_world ? "Mug joined to world" : "Mug joined to table");
    MultibodyPlant<double> plant(0.0);

    Parser parser(&plant, "test");
    const ModelInstanceIndex table_model =
        parser.AddModelsFromUrl(table_sdf_url).at(0);
    const RigidBody<double>& table = plant.GetBodyByName("link", table_model);
    const auto& prismatic_joint =
        plant.AddJoint(std::make_unique<PrismaticJoint<double>>(
            "table", plant.world_frame(), table.body_frame(),
            Vector3d(1, 1, 1)));
    EXPECT_FALSE(prismatic_joint.is_ephemeral());

    // Add a mug with no joint. This will become a floating base body at
    // finalization unless we add a joint.
    parser.AddModelsFromUrl(mug_sdf_url);
    const RigidBody<double>& mug = plant.GetBodyByName("simple_mug");
    if (!mug_in_world) {
      // Explicitly put a 6-dof joint between mug and table, making this a
      // "free" body, but _not_ a floating base body.
      const auto& joint =
          plant.AddJoint<QuaternionFloatingJoint>("sixdof", table, {}, mug, {});
      // An explicitly added joint is not ephemeral.
      EXPECT_FALSE(joint.is_ephemeral());
    }

    plant.Finalize();
    auto context = plant.CreateDefaultContext();

    // Table has non-identity position and velocity w.r.t. world.
    VectorX<double> table_q =
        plant.GetPositionsAndVelocities(*context, table.model_instance());
    DRAKE_DEMAND(table_q.size() == 2);
    table_q << 3, 3;
    plant.SetPositionsAndVelocities(context.get(), table.model_instance(),
                                    table_q);

    const RigidTransformd I = RigidTransformd::Identity();

    const RigidTransformd X_WT = plant.EvalBodyPoseInWorld(*context, table);
    const RigidTransformd X_PM(Vector3d(-3, 2, -1));
    const RigidTransformd X_WM = mug_in_world ? X_PM : X_WT * X_PM;
    // Mug is in the world, or its world and transform poses are different.
    DRAKE_DEMAND(mug_in_world || !CompareMatrices(X_PM.GetAsMatrix34(),
                                                  X_WM.GetAsMatrix34(), 1));
    // Whether the mug is a "floating base" body depends on its parent frame.
    ASSERT_EQ(mug.is_floating_base_body(), mug_in_world);
    ASSERT_EQ(plant.GetFloatingBaseBodies().contains(mug.index()),
              mug_in_world);

    plant.SetFreeBodyPose(context.get(), mug, X_PM);

    // The value we set for free body pose should always come right back -- it's
    // the literal configuration of the joint.
    EXPECT_TRUE(
        CompareMatrices(plant.GetFreeBodyPose(*context, mug).GetAsMatrix34(),
                        X_PM.GetAsMatrix34()));

    // The world pose is as expected.
    EXPECT_TRUE(CompareMatrices(
        plant.EvalBodyPoseInWorld(*context, mug).GetAsMatrix34(),
        X_WM.GetAsMatrix34()));

    // Reset the pose.
    plant.SetFreeBodyPose(context.get(), mug, I);
    EXPECT_FALSE(CompareMatrices(
        plant.EvalBodyPoseInWorld(*context, mug).GetAsMatrix34(),
        X_WM.GetAsMatrix34(), 1));

    // Note: We're not explicitly testing
    // MultibodyPlant::SetFreeBodyPose(context, state, body, X_PB) because the
    // non-state variant ultimately just forwards to the same API. MbP's
    // implementation is a tiny wrapper around the code that is ultimately
    // tested by the call to SetFreeBodyPose(context, body, X_PB).

    // Explicitly setting things in the world frame produces the expected pose
    // in world for floating base bodies, but not for "internal" free bodies.
    if (mug_in_world) {
      plant.SetFloatingBaseBodyPoseInWorldFrame(context.get(), mug, X_WM);
      EXPECT_TRUE(CompareMatrices(
          plant.EvalBodyPoseInWorld(*context, mug).GetAsMatrix34(),
          X_WM.GetAsMatrix34()));
    } else {
      EXPECT_THROW(
          plant.SetFloatingBaseBodyPoseInWorldFrame(context.get(), mug, X_WM),
          std::exception);
    }

    // Although the mug has a quaternion floating joint, that is insufficient
    // for setting a *default* floating pose -- the parent must be world.
    plant.SetDefaultFloatingBaseBodyPose(mug, X_PM);

    // As promised, the value set is regurgitated back.
    EXPECT_TRUE(CompareMatrices(
        plant.GetDefaultFloatingBaseBodyPose(mug).GetAsMatrix34(),
        X_PM.GetAsMatrix34()));

    // The default pose takes affect iff the world is the parent frame.
    auto alt_context = plant.CreateDefaultContext();
    EXPECT_EQ(CompareMatrices(
                  plant.GetFreeBodyPose(*alt_context, mug).GetAsMatrix34(),
                  X_PM.GetAsMatrix34()),
              mug_in_world);

    const SpatialVelocity<double> V_PB(Vector3d(1.0, 2.0, 3.0),
                                       Vector3d(-1.0, 4.0, -0.5));
    plant.SetFreeBodySpatialVelocity(context.get(), mug, V_PB);
    if (mug_in_world) {
      EXPECT_TRUE(CompareMatrices(
          plant.EvalBodySpatialVelocityInWorld(*context, mug).get_coeffs(),
          V_PB.get_coeffs()));
    } else {
      EXPECT_FALSE(CompareMatrices(
          plant.EvalBodySpatialVelocityInWorld(*context, mug).get_coeffs(),
          V_PB.get_coeffs(), 1.0));
    }

    // We omit the SetFreeBodySpatialVelocity() that takes state for the same
    // reason as omitting SetFreeBodyPose(): the thing wrapper can be validated
    // by inspection, the the overload called here tests the shared, underlying
    // implementation.
  }
}

// This unit test verifies the workings of
// MBP::SetFloatingBaseBodyPoseInAnchoredFrame(). To that end we build a model
// representative of a real setup consisting of a robot arm mounted on a robot
// table, an objects table and a mug. This test defines an objects frame O with
// its origin located a the -x, -y corner of the objects table. With this setup,
// we test we can set the pose X_OM of the mug frame M in the objects frame O.
GTEST_TEST(StateSelection, FloatingBodies) {
  const std::string iiwa_sdf_url =
      "package://drake_models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf";

  const std::string table_sdf_url =
      "package://drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf";

  const std::string mug_sdf_url =
      "package://drake/examples/simple_gripper/simple_mug.sdf";

  MultibodyPlant<double> plant(0.0);

  // Load a model of a table for the robot.
  Parser robot_parser(&plant, "robot");
  const ModelInstanceIndex robot_table_model =
      robot_parser.AddModelsFromUrl(table_sdf_url).at(0);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", robot_table_model));

  // Load the robot and weld it on top of the robot table.
  const ModelInstanceIndex arm_model =
      robot_parser.AddModelsFromUrl(iiwa_sdf_url).at(0);

  const double table_top_z_in_world =
      // table's top height
      0.736 +
      // table's top width
      0.057 / 2;
  const RigidTransformd X_WLink0(Vector3d(0, 0, table_top_z_in_world));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("iiwa_link_0", arm_model), X_WLink0);

  // Load a second table for objects.
  Parser objects_parser(&plant, "objects");
  const ModelInstanceIndex objects_table_model =
      objects_parser.AddModelsFromUrl(table_sdf_url).at(0);
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
  objects_parser.AddModelsFromUrl(mug_sdf_url);
  const RigidBody<double>& mug = plant.GetBodyByName("simple_mug");

  plant.Finalize();

  // Assert that the mug is a free body before moving on with this assumption.
  ASSERT_TRUE(mug.is_floating_base_body());
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
  EXPECT_FALSE(plant.world_body().is_floating_base_body());

  // Sanity check that bodies welded to the world are not free.
  EXPECT_FALSE(plant.GetBodyByName("iiwa_link_0").is_floating_base_body());
  EXPECT_FALSE(
      plant.GetBodyByName("link", objects_table_model).is_floating_base_body());

  std::unordered_set<BodyIndex> expected_floating_bodies({mug.index()});
  auto floating_bodies = plant.GetFloatingBaseBodies();
  EXPECT_EQ(expected_floating_bodies, floating_bodies);

  // Check link 0 is anchored, and link 1 is not.
  EXPECT_TRUE(plant.IsAnchored(plant.GetBodyByName("iiwa_link_0", arm_model)));
  EXPECT_FALSE(plant.IsAnchored(plant.GetBodyByName("iiwa_link_1", arm_model)));

  auto context = plant.CreateDefaultContext();

  // Initialize the pose X_OM of the mug frame M in the objects table frame O.
  const Vector3d p_OoMo_O(0.05, 0.0, 0.05);
  const RigidTransformd X_OM(p_OoMo_O);
  plant.SetFloatingBaseBodyPoseInAnchoredFrame(context.get(), objects_frame_O,
                                               mug, X_OM);

  // Retrieve the pose of the mug in the world.
  const RigidTransformd& X_WM = plant.EvalBodyPoseInWorld(*context, mug);

  const RigidTransformd X_WM_expected = X_WT * X_TO * X_OM;

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_WM.GetAsMatrix4(), X_WM_expected.GetAsMatrix4(),
                              kTolerance, MatrixCompareType::relative));

  // SetFloatingBaseBodyPoseInAnchoredFrame() should throw if the reference
  // frame F is not anchored to the world.
  const Frame<double>& end_effector_frame =
      plant.GetFrameByName("iiwa_link_7", arm_model);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.SetFloatingBaseBodyPoseInAnchoredFrame(
          context.get(), end_effector_frame, mug, X_OM),
      "Frame 'iiwa_link_7' must be anchored to the world frame.");

  // Check qdot to v mappings.
  VectorXd q = Eigen::VectorXd::LinSpaced(plant.num_positions(), -1, 1);
  VectorXd v = Eigen::VectorXd::LinSpaced(plant.num_velocities(), -2, 2);
  VectorXd qdot(plant.num_positions());
  VectorXd v_back(plant.num_velocities());
  plant.SetPositions(context.get(), q);
  plant.MapVelocityToQDot(*context, v, &qdot);
  plant.MapQDotToVelocity(*context, qdot, &v_back);
  EXPECT_TRUE(CompareMatrices(v, v_back, kTolerance));
  const Eigen::SparseMatrix<double> N = plant.MakeVelocityToQDotMap(*context);
  const Eigen::SparseMatrix<double> Nplus =
      plant.MakeQDotToVelocityMap(*context);
  EXPECT_TRUE(CompareMatrices(qdot, N * v, kTolerance));
  EXPECT_TRUE(CompareMatrices(v, Nplus * qdot, kTolerance));
}

// Verify that we can control what kind of joint is used to connect an
// unconnected body to World, globally or per-model instance. The default
// should be QuaternionFloatingJoint, with RpyFloating and Weld as options.
// These should all be marked "ephemeral" since they aren't user-added.
//
// We'll also verify that we can set state (q & v), pose, and velocity using the
// generic Joint API applied to the floating joints. We'll also check that we
// get reasonable messages when we misuse the API on a non-floating joint.
// Note: Set/GetPose() are just calls to Set/GetPosePair() so the latter aren't
// tested separately except to verify for a QuaternionFloatingJoint that
// the quaternion and translation are preserved bit-identically.
GTEST_TEST(MultibodyPlantTest, BaseBodyJointChoice) {
  // Plant will contain a free body in the default model instance and one
  // in a specific model instance. We need fresh plants for each case below.
  std::unique_ptr<MultibodyPlant<double>> plant;
  ModelInstanceIndex model_instance;
  const RigidBody<double>* default_body{};
  const RigidBody<double>* instance_body{};
  const auto make_plant = [&]() {
    plant = std::make_unique<MultibodyPlant<double>>(0.0);
    model_instance = plant->AddModelInstance("instance");
    default_body = &plant->AddRigidBody("DefaultBody", default_model_instance(),
                                        SpatialInertia<double>::MakeUnitary());
    instance_body = &plant->AddRigidBody("InstanceBody", model_instance,
                                         SpatialInertia<double>::MakeUnitary());
  };

  {  // Case 1: all defaults.
    make_plant();

    // Check the default global and model instance setting.
    EXPECT_EQ(plant->GetBaseBodyJointType(),
              BaseBodyJointType::kQuaternionFloatingJoint);
    EXPECT_EQ(plant->GetBaseBodyJointType(model_instance),
              BaseBodyJointType::kQuaternionFloatingJoint);

    plant->Finalize();  // Two quaternion floating joints.
    EXPECT_TRUE(default_body->is_floating_base_body());
    EXPECT_TRUE(default_body->has_quaternion_dofs());
    EXPECT_TRUE(instance_body->is_floating_base_body());
    EXPECT_TRUE(instance_body->has_quaternion_dofs());
    EXPECT_EQ(plant->num_joints(), 2);
    EXPECT_EQ(plant->num_positions(), 14);
    EXPECT_EQ(plant->num_velocities(), 12);

    // When ephemeral base joints are added they are named after the base body.
    const Joint<double>& quaternion_joint =
        plant->GetJointByName("InstanceBody");
    EXPECT_TRUE(quaternion_joint.is_ephemeral());
    auto context = plant->CreateDefaultContext();
    Eigen::Vector<double, 7> set_q;
    set_q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
    Vector6d set_v;
    set_v << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;
    quaternion_joint.SetPositions(&*context, set_q);
    quaternion_joint.SetVelocities(&*context, set_v);
    EXPECT_EQ(quaternion_joint.GetPositions(*context), set_q);
    EXPECT_EQ(quaternion_joint.GetVelocities(*context), set_v);

    EXPECT_FALSE(quaternion_joint.GetPose(*context).IsNearlyIdentity(1e-8));
    quaternion_joint.SetPose(&*context, RigidTransformd::Identity());
    EXPECT_TRUE(quaternion_joint.GetPose(*context).IsNearlyIdentity(1e-14));

    // Check that a quaternion floating joint preserves given
    // (quaternion, translation) bit-identically.
    const Eigen::Quaternion<double> set_quaternion(
        RollPitchYawd(0.01, 0.02, 0.03).ToQuaternion());
    const Vector3d set_translation(1.0, 2.0, 3.0);
    const RigidTransformd set_pose(set_quaternion, set_translation);
    quaternion_joint.SetPosePair(&*context, set_quaternion, set_translation);
    const std::pair<Eigen::Quaternion<double>, Vector3d> actual_pose =
        quaternion_joint.GetPosePair(*context);
    EXPECT_EQ(actual_pose.first.coeffs(), set_quaternion.coeffs());
    EXPECT_EQ(actual_pose.second, set_translation);

    const SpatialVelocity<double> zero(Vector6d::Zero());
    EXPECT_NE(quaternion_joint.GetSpatialVelocity(*context).get_coeffs(),
              zero.get_coeffs());
    quaternion_joint.SetSpatialVelocity(&*context, zero);
    EXPECT_EQ(quaternion_joint.GetSpatialVelocity(*context).get_coeffs(),
              zero.get_coeffs());
  }

  {  // Case 2: globally choose RpyFloating.
    make_plant();
    plant->SetBaseBodyJointType(BaseBodyJointType::kRpyFloatingJoint);
    plant->Finalize();  // Two rpy floating joints.

    // Check the default global and model instance setting. (Post finalize
    // here for a sanity check -- doesn't change anything.)
    EXPECT_EQ(plant->GetBaseBodyJointType(),
              BaseBodyJointType::kRpyFloatingJoint);
    EXPECT_EQ(plant->GetBaseBodyJointType(model_instance),
              BaseBodyJointType::kRpyFloatingJoint);

    EXPECT_TRUE(default_body->is_floating_base_body());
    EXPECT_FALSE(default_body->has_quaternion_dofs());
    EXPECT_TRUE(instance_body->is_floating_base_body());
    EXPECT_FALSE(instance_body->has_quaternion_dofs());
    EXPECT_EQ(plant->num_joints(), 2);
    EXPECT_EQ(plant->num_positions(), 12);
    EXPECT_EQ(plant->num_velocities(), 12);

    // When base joints are added they are named after the base body.
    const Joint<double>& instance_joint = plant->GetJointByName("InstanceBody");
    EXPECT_TRUE(instance_joint.is_ephemeral());
    auto context = plant->CreateDefaultContext();
    Vector6d set_q;
    set_q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    Vector6d set_v;
    set_v << 1.1, 1.2, 1.3, 1.4, 1.5, 1.6;
    instance_joint.SetPositions(&*context, set_q);
    instance_joint.SetVelocities(&*context, set_v);
    EXPECT_EQ(instance_joint.GetPositions(*context), set_q);
    EXPECT_EQ(instance_joint.GetVelocities(*context), set_v);

    EXPECT_FALSE(instance_joint.GetPose(*context).IsNearlyIdentity(1e-8));
    instance_joint.SetPose(&*context, RigidTransformd::Identity());
    EXPECT_TRUE(instance_joint.GetPose(*context).IsNearlyIdentity(1e-14));

    const SpatialVelocity<double> zero(Vector6d::Zero());
    EXPECT_NE(instance_joint.GetSpatialVelocity(*context).get_coeffs(),
              zero.get_coeffs());
    instance_joint.SetSpatialVelocity(&*context, zero);
    EXPECT_EQ(instance_joint.GetSpatialVelocity(*context).get_coeffs(),
              zero.get_coeffs());
  }

  {  // Case 3: choose RpyFloating only for model_instance.
    make_plant();
    plant->SetBaseBodyJointType(BaseBodyJointType::kRpyFloatingJoint,
                                model_instance);

    EXPECT_EQ(plant->GetBaseBodyJointType(),
              BaseBodyJointType::kQuaternionFloatingJoint);
    EXPECT_EQ(plant->GetBaseBodyJointType(model_instance),
              BaseBodyJointType::kRpyFloatingJoint);

    plant->Finalize();  // One quaternion and one rpy floating joint.
    EXPECT_TRUE(default_body->is_floating_base_body());
    EXPECT_TRUE(default_body->has_quaternion_dofs());
    EXPECT_TRUE(instance_body->is_floating_base_body());
    EXPECT_FALSE(instance_body->has_quaternion_dofs());
    EXPECT_EQ(plant->num_joints(), 2);
    EXPECT_EQ(plant->num_positions(), 13);
    EXPECT_EQ(plant->num_velocities(), 12);
  }

  {  // Case 4: choose Weld globally but override with QuaternionFloating for
     // model_instance.
    make_plant();
    plant->SetBaseBodyJointType(BaseBodyJointType::kWeldJoint);
    plant->SetBaseBodyJointType(BaseBodyJointType::kQuaternionFloatingJoint,
                                model_instance);

    EXPECT_EQ(plant->GetBaseBodyJointType(), BaseBodyJointType::kWeldJoint);
    EXPECT_EQ(plant->GetBaseBodyJointType(model_instance),
              BaseBodyJointType::kQuaternionFloatingJoint);

    plant->Finalize();  // One weld and one quaternion floating joint.
    EXPECT_FALSE(default_body->is_floating_base_body());
    EXPECT_FALSE(default_body->has_quaternion_dofs());
    EXPECT_TRUE(instance_body->is_floating_base_body());
    EXPECT_TRUE(instance_body->has_quaternion_dofs());
    EXPECT_EQ(plant->num_joints(), 2);
    EXPECT_EQ(plant->num_positions(), 7);
    EXPECT_EQ(plant->num_velocities(), 6);

    // We'll use the weld joint to generate some Joint API errors.
    const Joint<double>& weld_joint = plant->GetJointByName("DefaultBody");
    EXPECT_EQ(weld_joint.type_name(), "weld");
    EXPECT_TRUE(weld_joint.is_ephemeral());
    auto context = plant->CreateDefaultContext();

    // SetPositions() and SetVelocities() should work for every joint as
    // long as the given vector is the right size (0 for Welds). They should
    // fail with a reasonable message for a wrong-sized vector.
    const Eigen::Vector<double, 0> nothing;
    EXPECT_NO_THROW(weld_joint.SetPositions(&*context, nothing));
    EXPECT_NO_THROW(weld_joint.SetVelocities(&*context, nothing));
    DRAKE_EXPECT_THROWS_MESSAGE(
        weld_joint.SetPositions(&*context, Vector3d(1.0, 2.0, 3.0)),
        ".*SetPositions.*positions.size.*num_positions.*failed.*");
    DRAKE_EXPECT_THROWS_MESSAGE(
        weld_joint.SetVelocities(&*context, Vector3d(1.0, 2.0, 3.0)),
        ".*SetVelocities.*velocities.size.*num_velocities.*failed.*");

    // GetPose() and GetSpatialVelocity() should work, but currently the
    // Sets are only for floating joints.
    const SpatialVelocity<double> zero(Vector6d::Zero());
    EXPECT_TRUE(weld_joint.GetPose(*context).IsExactlyIdentity());
    EXPECT_EQ(weld_joint.GetSpatialVelocity(*context).get_coeffs(),
              zero.get_coeffs());

    DRAKE_EXPECT_THROWS_MESSAGE(
        weld_joint.SetPose(&*context, RigidTransformd::Identity()),
        ".*SetPose\\(\\).*weld joint does not implement.*"
        "joint 'DefaultBody'.*");

    DRAKE_EXPECT_THROWS_MESSAGE(
        weld_joint.SetSpatialVelocity(&*context, zero),
        ".*SetSpatialVelocity\\(\\).*weld joint does not implement.*"
        "joint 'DefaultBody'.*");
  }
}

GTEST_TEST(SetRandomTest, QuaternionFloatingBody) {
  // Create a model that contains a single body B.
  MultibodyPlant<double> plant(0.0);

  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& body =
      plant.AddRigidBody("LoneBody", SpatialInertia<double>::MakeUnitary());
  plant.Finalize();

  RandomGenerator generator;

  std::uniform_real_distribution<symbolic::Expression> uniform;
  Vector3<symbolic::Expression> xyz(1.0 + uniform(generator),
                                    2.0 + uniform(generator),
                                    3.0 + uniform(generator));

  plant.SetFreeBodyRandomTranslationDistribution(body, xyz);
  plant.SetFreeBodyRandomRotationDistributionToUniform(body);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.SetFreeBodyRandomAnglesDistribution(
          body, math::RollPitchYaw<symbolic::Expression>(1.0, 2.0, 3.0)),
      ".*Requires an rpy.*but free body LoneBody uses a quaternion.*");

  auto context = plant.CreateDefaultContext();

  const math::RigidTransform<double> X_WB_default =
      plant.GetFreeBodyPose(*context, body);

  plant.SetRandomContext(context.get(), &generator);
  math::RigidTransform<double> X_WB = plant.GetFreeBodyPose(*context, body);

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
  const math::RotationMatrix<double> R_WB_new(RollPitchYawd(0.3, 0.4, 0.5));
  plant.SetFreeBodyRandomRotationDistribution(
      body, R_WB_new.cast<symbolic::Expression>().ToQuaternion());

  plant.SetRandomContext(context.get(), &generator);
  X_WB = plant.GetFreeBodyPose(*context, body);

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(R_WB_new.matrix(), X_WB.rotation().matrix(),
                              kTolerance, MatrixCompareType::relative));
}

// Repeat the above test but with an RpyFloatingJoint rather than a
// QuaternionFloatingJoint. Translation is the same but orientation must
// be handled differently.
GTEST_TEST(SetRandomTest, RpyFloatingBody) {
  // Create a model that contains a single body B.
  MultibodyPlant<double> plant(0.0);

  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  const RigidBody<double>& body =
      plant.AddRigidBody("LoneBody", SpatialInertia<double>::MakeUnitary());
  plant.SetBaseBodyJointType(BaseBodyJointType::kRpyFloatingJoint);
  plant.Finalize();

  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;
  Vector3<symbolic::Expression> xyz(1.0 + uniform(generator),
                                    2.0 + uniform(generator),
                                    3.0 + uniform(generator));

  plant.SetFreeBodyRandomTranslationDistribution(body, xyz);
  plant.SetFreeBodyRandomAnglesDistribution(
      body, math::RollPitchYaw<symbolic::Expression>(xyz));

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.SetFreeBodyRandomRotationDistribution(
          body, math::RollPitchYaw<symbolic::Expression>(xyz).ToQuaternion()),
      ".*Requires a quaternion.*but free body LoneBody uses an rpy.*");

  auto context = plant.CreateDefaultContext();

  const math::RigidTransform<double> X_WB_default =
      plant.GetFreeBodyPose(*context, body);

  plant.SetRandomContext(context.get(), &generator);
  math::RigidTransform<double> X_WB = plant.GetFreeBodyPose(*context, body);

  // Just make sure that the rotation matrices have changed.
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
  const RollPitchYawd dist(0.3, 0.4, 0.5);
  const math::RollPitchYaw<symbolic::Expression> dist_symbolic(dist.vector());
  plant.SetFreeBodyRandomAnglesDistribution(body, dist_symbolic);

  plant.SetRandomContext(context.get(), &generator);
  X_WB = plant.GetFreeBodyPose(*context, body);

  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(X_WB.rotation().matrix(),
                              dist.ToRotationMatrix().matrix(), kTolerance,
                              MatrixCompareType::relative));
}

// The random positions generated by MbP should fall back to the default
// positions when no distribution is specified.
GTEST_TEST(SetRandomTest, SetDefaultWhenNoDistributionSpecified) {
  // Create a model that contains a free body and an acrobot.
  MultibodyPlant<double> plant(0.0);
  // Add a rigid body that is "implicitly floating" (i.e. no joint specified).
  const RigidBody<double>& body0 =
      plant.AddRigidBody("free body 0", SpatialInertia<double>::MakeUnitary());
  // Add a rigid body that is "explicitly floating" (i.e. floating joint
  // explicitly added).
  const RigidBody<double>& body1 =
      plant.AddRigidBody("free body 1", SpatialInertia<double>::MakeUnitary());
  plant.AddJoint<QuaternionFloatingJoint>(body1.name(), plant.world_body(), {},
                                          body1, {});
  const std::string acrobot_url =
      "package://drake/multibody/benchmarks/acrobot/acrobot.sdf";
  const ModelInstanceIndex acrobot =
      Parser(&plant).AddModelsFromUrl(acrobot_url)[0];
  plant.Finalize();

  // Set default positions through two different code paths.
  const Eigen::VectorXd acrobot_default_positions =
      Eigen::VectorXd::LinSpaced(2, 1.0, 2.0);  // 2 joints.
  plant.SetDefaultPositions(acrobot, acrobot_default_positions);
  const RigidTransformd X_WB0(RollPitchYawd(0.3, 0.4, 0.5),
                              Vector3d(1.0, 2.0, 3.0));
  const RigidTransformd X_WB1(RollPitchYawd(0.1, 0.2, 0.3),
                              Vector3d(2.0, 4.0, 6.0));
  plant.SetDefaultFloatingBaseBodyPose(body0, X_WB0);
  plant.SetDefaultFloatingBaseBodyPose(body1, X_WB1);

  // The random positions should match the default positions when no
  // distribution is set.
  auto context = plant.CreateDefaultContext();
  const Eigen::VectorXd expected_q = plant.GetPositions(*context);
  RandomGenerator generator;
  plant.SetRandomContext(context.get(), &generator);
  const Eigen::VectorXd q = plant.GetPositions(*context);
  EXPECT_EQ(q, expected_q);
}

GTEST_TEST(MultibodyPlantTest, SceneGraphPorts) {
  MultibodyPlant<double> plant(0.0);

  MultibodyPlant<double> plant_finalized(0.0);
  plant_finalized.Finalize();

  // Test that SceneGraph ports exist and are accessible, both pre and post
  // finalize, without the presence of a connected SceneGraph.
  EXPECT_NO_THROW(plant.get_geometry_query_input_port());
  EXPECT_NO_THROW(plant.get_geometry_pose_output_port());
  EXPECT_NO_THROW(plant_finalized.get_geometry_query_input_port());
  EXPECT_NO_THROW(plant_finalized.get_geometry_pose_output_port());
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
      UnitInertia<double>::SolidCube(cube_length);
  const RigidBody<double>& cube = plant.AddRigidBody(
      "cube", SpatialInertia<double>(cube_mass, cube_com, cube_unit_inertia));

  // These rigid bodies test the AddRigidBody overloads that do not specify any
  // inertia.
  const RigidBody<double>& simple_1 = plant.AddRigidBody("simple_1");
  const RigidBody<double>& simple_2 =
      plant.AddRigidBody("simple_2", default_model_instance());

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

  // The default inertia should be zero.
  for (const auto* simple : {&simple_1, &simple_2}) {
    EXPECT_GE(simple->get_mass(*context), 0.0);
    EXPECT_TRUE(
        simple->CalcSpatialInertiaInBodyFrame(*context).IsPhysicallyValid());
  }

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
      UnitInertia<double>::SolidCube(new_cube_length);

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
  EXPECT_TRUE(CompareMatrices(new_sphere_com_in_context, new_sphere_com));
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
  EXPECT_TRUE(CompareMatrices(new_cube_com_in_context, new_cube_com));
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

  elbow_joint_parent_frame.SetPoseInParentFrame(context_autodiff.get(),
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
  // clang-format off
  analytic_mass_matrix << I1 + I2 + m2*l1*l1 + 2*m2*l1*lc2, I2 + m2*l1*lc2,
                                            I2 + m2*l1*lc2,             I2;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(mass_matrix, analytic_mass_matrix, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂m₁:
  // [ (1/3)l₁²      0 ]
  // [     0         0 ]
  Vector4<double> analytic_mass_matrix_partial_m1;
  // clang-format off
  analytic_mass_matrix_partial_m1 << (1.0 / 3.0) * l1 * l1, 0.0,
                                                       0.0, 0.0;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(0),
                              analytic_mass_matrix_partial_m1, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂m₂:
  // [ (1/3)l₂² + l₁² + 2l₁lc₂c₂     (1/3)l₂² + l₁lc₂c₂ ]
  // [    (1/3)l₂² + l₁lc₂c₂               (1/3)l₂²     ]
  Vector4<double> analytic_mass_matrix_partial_m2;
  // clang-format off
  analytic_mass_matrix_partial_m2
      << (1.0/3.0)*l2*l2 + l1*l1 + 2*l1*lc2, (1.0/3.0)*l2*l2 + l1*lc2,
                   (1.0/3.0)*l2*l2 + l1*lc2,          (1.0/3.0)*l2*l2;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(1),
                              analytic_mass_matrix_partial_m2, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂l₁:
  //  [ (2/3)m₁l₁ + 2m₂l₁ + 2m₂lc₂c₂   m₂lc₂c₂ ]
  //  [        m₂lc₂c₂                    0    ]
  Vector4<double> analytic_mass_matrix_partial_l1;
  // clang-format off
  analytic_mass_matrix_partial_l1
      << (2.0/3.0)*m1*l1 + 2*m2*l1 + 2*m2*lc2, m2*lc2,
                                       m2*lc2,      0;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(2),
                              analytic_mass_matrix_partial_l1, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂l₂:
  // [   (2/3)m₂l₂ + m₂l₁       (2/3)m₂l₂ + (1/2)m₂l₁ ]
  // [ (2/3)m₂l₂ + (1/2)m₂l₁          (2/3)m₂l₂       ]
  Vector4<double> analytic_mass_matrix_partial_l2;
  // clang-format off
  analytic_mass_matrix_partial_l2 <<
            (2.0/3.0)*m2*l2 + m2*l1, (2.0/3.0)*m2*l2 + 0.5*m2*l1,
        (2.0/3.0)*m2*l2 + 0.5*m2*l1,             (2.0/3.0)*m2*l2;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(3),
                              analytic_mass_matrix_partial_l2, kTolerance,
                              MatrixCompareType::relative));
}

class MultibodyPlantConstraintTestTimeStepParam
    : public ::testing::TestWithParam<double> {
 public:
  MultibodyPlantConstraintTestTimeStepParam()
      : plant_(GetParam()),
        body_A_(plant_.AddRigidBody("body_A", SpatialInertia<double>::NaN())),
        body_B_(plant_.AddRigidBody("body_B", SpatialInertia<double>::NaN())) {}

 protected:
  MultibodyPlant<double> plant_;
  const RigidBody<double>& body_A_;
  const RigidBody<double>& body_B_;
};

TEST_P(MultibodyPlantConstraintTestTimeStepParam, GetConstraintIds) {
  // Set up a plant with each constraint type with arbitrary parameters.
  EXPECT_EQ(plant_.GetConstraintIds().size(), 0);
  const RevoluteJoint<double>& world_A = plant_.AddJoint<RevoluteJoint>(
      "world_A", plant_.world_body(), std::nullopt, body_A_, std::nullopt,
      Vector3d::UnitZ());
  const RevoluteJoint<double>& A_B = plant_.AddJoint<RevoluteJoint>(
      "A_B", body_A_, std::nullopt, body_B_, std::nullopt, Vector3d::UnitZ());
  MultibodyConstraintId coupler_id =
      plant_.AddCouplerConstraint(world_A, A_B, 2.3);
  MultibodyConstraintId distance_id = plant_.AddDistanceConstraint(
      body_A_, Vector3d(1.0, 2.0, 3.0), body_B_, Vector3d(4.0, 5.0, 6.0), 2.0);
  MultibodyConstraintId ball_id = plant_.AddBallConstraint(
      body_A_, Vector3d(-1.0, -2.0, -3.0), body_B_, Vector3d(-4.0, -5.0, -6.0));
  MultibodyConstraintId weld_id = plant_.AddWeldConstraint(
      body_A_, RigidTransformd(), body_B_, RigidTransformd());
  MultibodyConstraintId tendon_id = plant_.AddTendonConstraint(
      {world_A.index()}, {1.0}, {2.0}, {-3.0}, {4.0}, {5.0}, {6.0});

  std::vector<MultibodyConstraintId> ids = plant_.GetConstraintIds();
  // The order of the constraints is not guaranteed.
  EXPECT_THAT(ids, testing::UnorderedElementsAre(coupler_id, distance_id,
                                                 ball_id, weld_id, tendon_id));

  plant_.RemoveConstraint(coupler_id);
  plant_.RemoveConstraint(ball_id);
  ids = plant_.GetConstraintIds();
  EXPECT_THAT(ids,
              testing::UnorderedElementsAre(distance_id, weld_id, tendon_id));
}

TEST_P(MultibodyPlantConstraintTestTimeStepParam, ConstraintActiveStatus) {
  // Set up a plant with 3 constraints with arbitrary parameters.
  const RevoluteJoint<double>& world_A = plant_.AddJoint<RevoluteJoint>(
      "world_A", plant_.world_body(), std::nullopt, body_A_, std::nullopt,
      Vector3d::UnitZ());
  const RevoluteJoint<double>& A_B = plant_.AddJoint<RevoluteJoint>(
      "A_B", body_A_, std::nullopt, body_B_, std::nullopt, Vector3d::UnitZ());
  MultibodyConstraintId coupler_id =
      plant_.AddCouplerConstraint(world_A, A_B, 2.3);
  MultibodyConstraintId distance_id = plant_.AddDistanceConstraint(
      body_A_, Vector3d(1.0, 2.0, 3.0), body_B_, Vector3d(4.0, 5.0, 6.0), 2.0);
  MultibodyConstraintId ball_id = plant_.AddBallConstraint(
      body_A_, Vector3d(-1.0, -2.0, -3.0), body_B_, Vector3d(-4.0, -5.0, -6.0));
  MultibodyConstraintId weld_id = plant_.AddWeldConstraint(
      body_A_, RigidTransformd(), body_B_, RigidTransformd());
  MultibodyConstraintId tendon_id = plant_.AddTendonConstraint(
      {world_A.index()}, {1.0}, {2.0}, {-3.0}, {4.0}, {5.0}, {6.0});

// Remove on 2026-09-01 per TAMSI deprecation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  if (plant_.is_discrete()) {
    DRAKE_EXPECT_THROWS_MESSAGE(plant_.set_discrete_contact_approximation(
                                    DiscreteContactApproximation::kTamsi),
                                ".*TAMSI does not support constraints.*");
  }
#pragma GCC diagnostic pop

  plant_.Finalize();

  std::unique_ptr<Context<double>> context = plant_.CreateDefaultContext();

  // Verify all constraints are active in a default context.
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, coupler_id));
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, distance_id));
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, ball_id));
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, weld_id));
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, tendon_id));

  // Set all constraints to inactive.
  plant_.SetConstraintActiveStatus(context.get(), coupler_id, false);
  plant_.SetConstraintActiveStatus(context.get(), distance_id, false);
  plant_.SetConstraintActiveStatus(context.get(), ball_id, false);
  plant_.SetConstraintActiveStatus(context.get(), weld_id, false);
  plant_.SetConstraintActiveStatus(context.get(), tendon_id, false);

  // Verify all constraints are inactive in the context.
  EXPECT_FALSE(plant_.GetConstraintActiveStatus(*context, coupler_id));
  EXPECT_FALSE(plant_.GetConstraintActiveStatus(*context, distance_id));
  EXPECT_FALSE(plant_.GetConstraintActiveStatus(*context, ball_id));
  EXPECT_FALSE(plant_.GetConstraintActiveStatus(*context, weld_id));
  EXPECT_FALSE(plant_.GetConstraintActiveStatus(*context, tendon_id));

  // Set all constraints to back to active.
  plant_.SetConstraintActiveStatus(context.get(), coupler_id, true);
  plant_.SetConstraintActiveStatus(context.get(), distance_id, true);
  plant_.SetConstraintActiveStatus(context.get(), ball_id, true);
  plant_.SetConstraintActiveStatus(context.get(), weld_id, true);
  plant_.SetConstraintActiveStatus(context.get(), tendon_id, true);

  // Verify all constraints are active in the context.
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, coupler_id));
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, distance_id));
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, ball_id));
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, weld_id));
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, weld_id));
  EXPECT_TRUE(plant_.GetConstraintActiveStatus(*context, tendon_id));
}

TEST_P(MultibodyPlantConstraintTestTimeStepParam, RemoveConstraint) {
  // Set up a plant with 3 constraints with arbitrary parameters.
  const RevoluteJoint<double>& world_A = plant_.AddJoint<RevoluteJoint>(
      "world_A", plant_.world_body(), std::nullopt, body_A_, std::nullopt,
      Vector3d::UnitZ());
  const RevoluteJoint<double>& A_B = plant_.AddJoint<RevoluteJoint>(
      "A_B", body_A_, std::nullopt, body_B_, std::nullopt, Vector3d::UnitZ());
  MultibodyConstraintId coupler_id =
      plant_.AddCouplerConstraint(world_A, A_B, 2.3);
  MultibodyConstraintId distance_id = plant_.AddDistanceConstraint(
      body_A_, Vector3d(1.0, 2.0, 3.0), body_B_, Vector3d(4.0, 5.0, 6.0), 2.0);
  MultibodyConstraintId ball_id = plant_.AddBallConstraint(
      body_A_, Vector3d(-1.0, -2.0, -3.0), body_B_, Vector3d(-4.0, -5.0, -6.0));
  MultibodyConstraintId weld_id = plant_.AddWeldConstraint(
      body_A_, RigidTransformd(), body_B_, RigidTransformd());
  MultibodyConstraintId tendon_id = plant_.AddTendonConstraint(
      {world_A.index()}, {1.0}, {2.0}, {-3.0}, {4.0}, {5.0}, {6.0});

  EXPECT_EQ(plant_.num_coupler_constraints(), 1);
  EXPECT_EQ(plant_.num_distance_constraints(), 1);
  EXPECT_EQ(plant_.num_ball_constraints(), 1);
  EXPECT_EQ(plant_.num_weld_constraints(), 1);
  EXPECT_EQ(plant_.num_tendon_constraints(), 1);
  plant_.RemoveConstraint(coupler_id);
  plant_.RemoveConstraint(distance_id);
  plant_.RemoveConstraint(ball_id);
  plant_.RemoveConstraint(weld_id);
  plant_.RemoveConstraint(tendon_id);
  EXPECT_EQ(plant_.num_coupler_constraints(), 0);
  EXPECT_EQ(plant_.num_distance_constraints(), 0);
  EXPECT_EQ(plant_.num_ball_constraints(), 0);
  EXPECT_EQ(plant_.num_weld_constraints(), 0);
  EXPECT_EQ(plant_.num_tendon_constraints(), 0);

  DRAKE_EXPECT_THROWS_MESSAGE(plant_.RemoveConstraint(coupler_id),
                              ".*does not match any constraint.*");

  // Add a new coupler constraint
  MultibodyConstraintId coupler_id2 =
      plant_.AddCouplerConstraint(world_A, A_B, 2.3);
  EXPECT_EQ(plant_.num_coupler_constraints(), 1);

  plant_.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_.RemoveConstraint(coupler_id2),
      ".*Post-finalize calls to .*RemoveConstraint.* are not allowed.*");
}

TEST_P(MultibodyPlantConstraintTestTimeStepParam, FinalizeConstraints) {
  // Set up a plant with partially specified constraints that must be
  // finalized.
  RigidTransformd X_WA(Vector3d(-1.0, -2.0, -3.0));
  plant_.AddJoint<RevoluteJoint>("world_A", plant_.world_body(), X_WA, body_A_,
                                 std::nullopt, Vector3d::UnitZ());
  RigidTransformd X_WB(Vector3d(1.2, 3.4, 5.6));
  plant_.WeldFrames(plant_.world_frame(), plant_.GetFrameByName("body_B"),
                    X_WB);

  Vector3d p_AP = Vector3d(4.0, 5.0, 6.0);
  MultibodyConstraintId ball_id = plant_.AddBallConstraint(
      body_A_, p_AP, body_B_, std::nullopt /* p_BQ is left unspecified */);
  EXPECT_FALSE(plant_.get_ball_constraint_specs(ball_id).p_BQ.has_value());

  plant_.Finalize();

  Vector3d p_BQ = X_WB.inverse() * X_WA * p_AP;  // since Q == P.
  ASSERT_TRUE(plant_.get_ball_constraint_specs(ball_id).p_BQ.has_value());
  EXPECT_TRUE(CompareMatrices(
      plant_.get_ball_constraint_specs(ball_id).p_BQ.value(), p_BQ, 1e-14));
}

GTEST_TEST(MultibodyPlantTests, FixedOffsetFrameFunctions) {
  MultibodyPlant<double> plant(0.0);
  const RigidBody<double>& body_B =
      plant.AddRigidBody("body_B", SpatialInertia<double>::NaN());

  // Weld body B to the world W which creates a fixed offset frame Wp fixed to
  // the world and a fixed offset frame P fixed to body B.
  const RigidTransformd X_WWp(RotationMatrixd::MakeZRotation(M_PI / 6),
                              Vector3d(2, 2, 0));
  const RigidTransformd X_WpP(RotationMatrixd::MakeZRotation(M_PI / 12),
                              Vector3d(0.1, 0.2, 0));
  RigidTransformd X_BP(RotationMatrixd::MakeXRotation(M_PI / 4),
                       Vector3d(0, 0.3, 0.4));
  const Joint<double>& weld_joint = plant.AddJoint<WeldJoint>(
      "weld_WB", plant.world_body(), X_WWp, body_B, X_BP, X_WpP);

  // Get a reference to the fixed offset frame P on body B.
  const FixedOffsetFrame<double>& frame_P =
      dynamic_cast<const FixedOffsetFrame<double>&>(
          weld_joint.frame_on_child());

  // Create a fixed offset frame F on body B whose parent frame is P.
  RigidTransformd X_PF(RotationMatrixd::MakeZRotation(M_PI / 3),
                       Vector3d(0.9, 0.7, 0.3));
  const FixedOffsetFrame<double>& frame_F =
      plant.AddFrame(std::make_unique<FixedOffsetFrame<double>>(
          "fixed_offset_frame_F", frame_P, X_PF));

  // Finalize the plant and create a default context.
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  constexpr double kTolerance = 8 * std::numeric_limits<double>::epsilon();

  // Verify frame P's pose in its parent B (reminder P is a fixed-offset frame).
  RigidTransformd X_BP_check1 = frame_P.GetPoseInParentFrame(*context);
  RigidTransformd X_BP_check2 = frame_P.CalcPoseInBodyFrame(*context);
  EXPECT_TRUE(X_BP_check1.IsNearlyEqualTo(X_BP, kTolerance));
  EXPECT_TRUE(X_BP_check2.IsNearlyEqualTo(X_BP, kTolerance));
  // Verify parent frame is body_B's frame
  EXPECT_EQ(frame_P.parent_frame().index(), body_B.body_frame().index());

  // Verify frame P's pose in world W.
  RigidTransformd X_WP = X_WWp * X_WpP;
  RigidTransformd X_WP_check = frame_P.CalcPoseInWorld(*context);
  EXPECT_TRUE(X_WP_check.IsNearlyEqualTo(X_WP, kTolerance));

  // Verify frame F's pose in its parent P.
  RigidTransformd X_PF_check = frame_F.GetPoseInParentFrame(*context);
  EXPECT_TRUE(X_PF_check.IsNearlyEqualTo(X_PF, kTolerance));

  // Verify frame F's pose in body B.
  RigidTransformd X_BF = X_BP * X_PF;
  RigidTransformd X_BF_check = frame_F.CalcPoseInBodyFrame(*context);
  EXPECT_TRUE(X_BF_check.IsNearlyEqualTo(X_BF, kTolerance));

  // Verify frame F's pose in world W.
  RigidTransformd X_WF = X_WWp * X_WpP * X_PF;
  RigidTransformd X_WF_check = frame_F.CalcPoseInWorld(*context);
  EXPECT_TRUE(X_WF_check.IsNearlyEqualTo(X_WF, kTolerance));

  // Verify body B's pose in world W.
  RigidTransformd X_WB = X_WWp * X_WpP * X_BP.inverse();
  RigidTransformd X_WB_check = plant.EvalBodyPoseInWorld(*context, body_B);
  EXPECT_TRUE(X_WB_check.IsNearlyEqualTo(X_WB, kTolerance));

  //-------------------------------------------------------------------------
  // Set new pose for fixed offset frame P and verify it propagates correctly.
  X_BP = RigidTransformd(RotationMatrixd::MakeXRotation(3 * M_PI / 4),
                         Vector3d(0, 0.5, 0.8));
  frame_P.SetPoseInParentFrame(context.get(), X_BP);

  // Verify frame P's new pose in its parent B.
  X_BP_check1 = frame_P.GetPoseInParentFrame(*context);
  X_BP_check2 = frame_P.CalcPoseInBodyFrame(*context);
  EXPECT_TRUE(X_BP_check1.IsNearlyEqualTo(X_BP, kTolerance));
  EXPECT_TRUE(X_BP_check2.IsNearlyEqualTo(X_BP, kTolerance));

  // Verify frame P's new pose in world W (which should not have changed).
  X_WP_check = frame_P.CalcPoseInWorld(*context);
  EXPECT_TRUE(X_WP_check.IsNearlyEqualTo(X_WP, kTolerance));

  //-------------------------------------------------------------------------
  // Set new pose for fixed offset frame F and verify it propagates correctly.
  X_PF = RigidTransformd(RotationMatrixd::MakeZRotation(2 * M_PI / 3),
                         Vector3d(2, 2, 2));
  frame_F.SetPoseInParentFrame(context.get(), X_PF);

  // Verify frame F's new pose in its parent P.
  X_PF_check = frame_F.GetPoseInParentFrame(*context);
  EXPECT_TRUE(X_PF_check.IsNearlyEqualTo(X_PF, kTolerance));

  // Verify frame F's new pose in body B.
  X_BF = X_BP * X_PF;
  X_BF_check = frame_F.CalcPoseInBodyFrame(*context);
  EXPECT_TRUE(X_BF_check.IsNearlyEqualTo(X_BF, kTolerance));

  // Verify frame F's new pose in world W.
  X_WF = X_WWp * X_WpP * X_PF;
  X_WF_check = frame_F.CalcPoseInWorld(*context);
  EXPECT_TRUE(X_WF_check.IsNearlyEqualTo(X_WF, kTolerance));

  // Verify body B's pose in world W.
  X_WB = X_WWp * X_WpP * X_BP.inverse();
  X_WB_check = plant.EvalBodyPoseInWorld(*context, body_B);
  EXPECT_TRUE(X_WB_check.IsNearlyEqualTo(X_WB, kTolerance));
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
  Parser(&plant).AddModelsFromUrl(
      "package://drake/multibody/plant/test/split_pendulum.sdf");
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

// Tests that the connecting to the actuation port for all instances is
// equivalent to connecting to the individual model instance actuation ports.
GTEST_TEST(MultibodyPlantTests, ActuationPorts) {
  const AcrobotParameters parameters;
  unique_ptr<MultibodyPlant<double>> plant =
      MakeAcrobotPlant(parameters, false /* Don't make a finalized plant. */);
  // Add a split pendulum to the plant.
  const ModelInstanceIndex pendulum_model_instance =
      Parser(plant.get())
          .AddModelsFromUrl(
              "package://drake/multibody/plant/test/split_pendulum.sdf")
          .at(0);
  plant->Finalize();
  ASSERT_EQ(plant->num_actuated_dofs(default_model_instance()), 1);
  ASSERT_EQ(plant->num_actuated_dofs(pendulum_model_instance), 1);
  ASSERT_EQ(plant->num_actuated_dofs(), 2);

  // Connect each actuation port separately.
  auto context1 = plant->CreateDefaultContext();
  plant->get_actuation_input_port(default_model_instance())
      .FixValue(context1.get(), 1.0);
  plant->get_actuation_input_port(pendulum_model_instance)
      .FixValue(context1.get(), 2.0);
  const auto& reaction_forces =
      plant->get_reaction_forces_output_port()
          .Eval<std::vector<SpatialForce<double>>>(*context1);

  // Connect the actuation port for all instances.
  auto context2 = plant->CreateDefaultContext();
  plant->get_actuation_input_port().FixValue(context2.get(),
                                             Vector2d(1.0, 2.0));
  const auto& expected_reaction_forces =
      plant->get_reaction_forces_output_port()
          .Eval<std::vector<SpatialForce<double>>>(*context2);

  // Indirectly verify that the actuation are the same by verifying the reaction
  // forces are the same and are non-zero.
  EXPECT_EQ(expected_reaction_forces.size(), reaction_forces.size());
  bool all_reaction_forces_zero = true;
  double kTolerance = 0.1;
  for (int i = 0; i < static_cast<int>(reaction_forces.size()); ++i) {
    EXPECT_TRUE(CompareMatrices(reaction_forces[i].translational(),
                                expected_reaction_forces[i].translational()));
    EXPECT_TRUE(CompareMatrices(reaction_forces[i].rotational(),
                                expected_reaction_forces[i].rotational()));
    all_reaction_forces_zero &=
        (reaction_forces[i].rotational().norm() < kTolerance);
  }
  EXPECT_FALSE(all_reaction_forces_zero);
}

// Verifies that a nice error message is thrown if actuation input port contains
// a NaN value.
GTEST_TEST(MultibodyPlantTest, ThrowIfActuationPortContainsNaN) {
  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeAcrobotPlant(parameters, true /* Return a finalized plant */);
  VectorXd nan_vector(plant->num_actuated_dofs());
  nan_vector.setConstant(std::numeric_limits<double>::quiet_NaN());
  auto context = plant->CreateDefaultContext();
  plant->get_actuation_input_port().FixValue(context.get(), nan_vector);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->get_reaction_forces_output_port()
          .Eval<std::vector<SpatialForce<double>>>(*context),
      ".*NaN in the actuation input port for all instances.*");
}

// Verifies that a nice error message is thrown if actuation input port for a
// certain model instance contains a NaN value.
GTEST_TEST(MultibodyPlantTest, ThrowIfModelInstanceActuationPortContainsNaN) {
  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeAcrobotPlant(parameters, true /* Return a finalized plant */);
  VectorXd nan_vector(plant->num_actuated_dofs());
  nan_vector.setConstant(std::numeric_limits<double>::quiet_NaN());
  auto context = plant->CreateDefaultContext();
  plant->get_actuation_input_port(default_model_instance())
      .FixValue(context.get(), nan_vector);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->get_reaction_forces_output_port()
          .Eval<std::vector<SpatialForce<double>>>(*context),
      "Actuation.*instance.*contains NaN.");
}

// Verifies that a nice error message is thrown if applied generalized force
// input port contains a NaN value.
GTEST_TEST(MultibodyPlantTest, ThrowIfGeneralizedForcePortContainsNaN) {
  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeAcrobotPlant(parameters, true /* Return a finalized plant */);
  VectorXd nan_vector(plant->num_velocities());
  nan_vector.setConstant(std::numeric_limits<double>::quiet_NaN());
  auto context = plant->CreateDefaultContext();
  plant->get_applied_generalized_force_input_port().FixValue(context.get(),
                                                             nan_vector);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->get_reaction_forces_output_port()
          .Eval<std::vector<SpatialForce<double>>>(*context),
      ".*NaN in applied generalized force.*");
}

// Verifies that a nice error message is thrown if applied spatial force
// input port contains a NaN value.
GTEST_TEST(MultibodyPlantTest, ThrowIfSpatialForcePortContainsNaN) {
  const AcrobotParameters parameters;
  std::unique_ptr<MultibodyPlant<double>> plant =
      MakeAcrobotPlant(parameters, true /* Return a finalized plant */);
  std::vector<ExternallyAppliedSpatialForce<double>> nan_spatial_force(1);
  nan_spatial_force[0].F_Bq_W.SetNaN();
  nan_spatial_force[0].body_index =
      plant->GetBodyByName(parameters.link1_name()).index();
  auto context = plant->CreateDefaultContext();
  plant->get_applied_spatial_force_input_port().FixValue(context.get(),
                                                         nan_spatial_force);
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->get_reaction_forces_output_port()
          .Eval<std::vector<SpatialForce<double>>>(*context),
      "Spatial force.*contains NaN.");
}

GTEST_TEST(MultibodyPlantTest, SetDefaultPositions) {
  // Construct a plant with two Iiwas.
  const char kSdfUrl[] =
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
  auto plant =
      std::make_unique<MultibodyPlant<double>>(0 /* plant type irrelevant */);
  multibody::ModelInstanceIndex iiwa0_instance =
      Parser(plant.get(), "iiwa0").AddModelsFromUrl(kSdfUrl).at(0);
  multibody::ModelInstanceIndex iiwa1_instance =
      Parser(plant.get(), "iiwa1").AddModelsFromUrl(kSdfUrl).at(0);
  // Weld iiwa0 to the world, leave iiwa1 to be floating.
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", iiwa0_instance));

  Eigen::VectorXd q = Eigen::VectorXd::LinSpaced(
      7 + 7 + 7, 1.0, 2.0);     // 7 joints each + 7 floating base positions.
  q.segment(7, 4).normalize();  // normalize the quaternion indices.

  // Throws if called pre-finalize.
  DRAKE_EXPECT_THROWS_MESSAGE(plant->SetDefaultPositions(q),
                              ".*you must call Finalize.* first.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->SetDefaultPositions(iiwa0_instance, q.head<7>()),
      ".*you must call Finalize.* first.");
  DRAKE_EXPECT_THROWS_MESSAGE(plant->GetDefaultPositions(),
                              ".*you must call Finalize.* first.");
  DRAKE_EXPECT_THROWS_MESSAGE(plant->GetDefaultPositions(iiwa0_instance),
                              ".*you must call Finalize.* first.");
  plant->Finalize();

  // Throws if the q is the wrong size.
  DRAKE_EXPECT_THROWS_MESSAGE(plant->SetDefaultPositions(q.head<3>()),
                              ".*num_positions.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant->SetDefaultPositions(iiwa1_instance, q.head<3>()),
      ".*q_instance.size.* == num_positions.*");

  const double kTol = 1e-15;

  EXPECT_FALSE(CompareMatrices(plant->GetDefaultPositions(), q, kTol));
  EXPECT_TRUE(
      CompareMatrices(plant->GetDefaultPositions(),
                      plant->GetPositions(*plant->CreateDefaultContext())));
  plant->SetDefaultPositions(q);
  EXPECT_TRUE(CompareMatrices(plant->GetDefaultPositions(), q, kTol));
  EXPECT_TRUE(
      CompareMatrices(plant->GetDefaultPositions(),
                      plant->GetPositions(*plant->CreateDefaultContext())));

  auto context = plant->CreateDefaultContext();
  EXPECT_TRUE(CompareMatrices(plant->GetPositions(*context), q, kTol));
  EXPECT_TRUE(CompareMatrices(plant->GetPositions(*context, iiwa0_instance),
                              q.head<7>()));
  EXPECT_TRUE(CompareMatrices(plant->GetPositions(*context, iiwa1_instance),
                              q.tail<14>(), kTol));

  // Change q, and now verify the model_instance variant.
  q = Eigen::VectorXd::LinSpaced(
      7 + 7 + 7, 3.0, 4.0);     // 7 joints each + 7 floating base positions.
  q.segment(7, 4).normalize();  // normalize the quaternion indices.

  EXPECT_FALSE(CompareMatrices(plant->GetDefaultPositions(iiwa0_instance),
                               q.head<7>(), kTol));
  plant->SetDefaultPositions(iiwa0_instance, q.head<7>());
  EXPECT_TRUE(CompareMatrices(plant->GetDefaultPositions(iiwa0_instance),
                              q.head<7>(), kTol));
  EXPECT_TRUE(
      CompareMatrices(plant->GetDefaultPositions(),
                      plant->GetPositions(*plant->CreateDefaultContext())));

  EXPECT_FALSE(CompareMatrices(plant->GetDefaultPositions(iiwa1_instance),
                               q.tail<14>(), kTol));
  plant->SetDefaultPositions(iiwa1_instance, q.tail<14>());
  EXPECT_TRUE(CompareMatrices(plant->GetDefaultPositions(iiwa1_instance),
                              q.tail<14>(), kTol));
  EXPECT_TRUE(
      CompareMatrices(plant->GetDefaultPositions(),
                      plant->GetPositions(*plant->CreateDefaultContext())));

  plant->SetDefaultContext(context.get());
  EXPECT_TRUE(CompareMatrices(plant->GetPositions(*context), q, kTol));
  EXPECT_TRUE(CompareMatrices(plant->GetPositions(*context, iiwa0_instance),
                              q.head<7>()));
  EXPECT_TRUE(CompareMatrices(plant->GetPositions(*context, iiwa1_instance),
                              q.tail<14>(), kTol));
  EXPECT_TRUE(
      CompareMatrices(plant->GetDefaultPositions(),
                      plant->GetPositions(*plant->CreateDefaultContext())));
}

GTEST_TEST(MultibodyPlantTest, GetNames) {
  // Construct a plant with two Iiwas.
  const char kSdfUrl[] =
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
  auto plant =
      std::make_unique<MultibodyPlant<double>>(0 /* plant type irrelevant */);
  multibody::ModelInstanceIndex iiwa0_instance =
      Parser(plant.get(), "iiwa0").AddModelsFromUrl(kSdfUrl).at(0);
  multibody::ModelInstanceIndex iiwa1_instance =
      Parser(plant.get(), "iiwa1").AddModelsFromUrl(kSdfUrl).at(0);
  // Weld iiwa0 to the world, leave iiwa1 to be floating.
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0", iiwa0_instance));

  // Throws if called pre-finalize.
  DRAKE_EXPECT_THROWS_MESSAGE(plant->GetPositionNames(),
                              ".*you must call Finalize.* first.");
  DRAKE_EXPECT_THROWS_MESSAGE(plant->GetVelocityNames(),
                              ".*you must call Finalize.* first.");
  DRAKE_EXPECT_THROWS_MESSAGE(plant->GetStateNames(),
                              ".*you must call Finalize.* first.");
  DRAKE_EXPECT_THROWS_MESSAGE(plant->GetActuatorNames(),
                              ".*you must call Finalize.* first.");
  plant->Finalize();

  // Throws if model_instance is invalid. Currently the message is platform
  // dependent (thrown by model_instances.at() in
  // MultibodyTree::num_positions(model_instance)).
  // TODO(russt): Throw a proper message in MultibodyTree.
  EXPECT_THROW(plant->GetPositionNames(ModelInstanceIndex()), std::exception);
  EXPECT_THROW(plant->GetVelocityNames(ModelInstanceIndex()), std::exception);
  EXPECT_THROW(plant->GetStateNames(ModelInstanceIndex()), std::exception);
  EXPECT_THROW(plant->GetActuatorNames(ModelInstanceIndex()), std::exception);

  std::vector<std::string> names = plant->GetPositionNames(iiwa0_instance);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_joint_1_q", "iiwa_joint_2_q", "iiwa_joint_3_q",
                          "iiwa_joint_4_q", "iiwa_joint_5_q", "iiwa_joint_6_q",
                          "iiwa_joint_7_q"}));

  names = plant->GetPositionNames(iiwa0_instance,
                                  true /* add_model_instance_prefix */,
                                  true /* always_add_suffix */);
  EXPECT_THAT(
      names,
      testing::ElementsAreArray(
          {"iiwa0::iiwa14_iiwa_joint_1_q", "iiwa0::iiwa14_iiwa_joint_2_q",
           "iiwa0::iiwa14_iiwa_joint_3_q", "iiwa0::iiwa14_iiwa_joint_4_q",
           "iiwa0::iiwa14_iiwa_joint_5_q", "iiwa0::iiwa14_iiwa_joint_6_q",
           "iiwa0::iiwa14_iiwa_joint_7_q"}));

  names = plant->GetPositionNames(iiwa0_instance,
                                  true /* add_model_instance_prefix */,
                                  false /* always_add_suffix */);
  EXPECT_THAT(names,
              testing::ElementsAreArray(
                  {"iiwa0::iiwa14_iiwa_joint_1", "iiwa0::iiwa14_iiwa_joint_2",
                   "iiwa0::iiwa14_iiwa_joint_3", "iiwa0::iiwa14_iiwa_joint_4",
                   "iiwa0::iiwa14_iiwa_joint_5", "iiwa0::iiwa14_iiwa_joint_6",
                   "iiwa0::iiwa14_iiwa_joint_7"}));

  names = plant->GetPositionNames(iiwa0_instance,
                                  false /* add_model_instance_prefix */,
                                  true /* always_add_suffix */);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_joint_1_q", "iiwa_joint_2_q", "iiwa_joint_3_q",
                          "iiwa_joint_4_q", "iiwa_joint_5_q", "iiwa_joint_6_q",
                          "iiwa_joint_7_q"}));

  names = plant->GetPositionNames(iiwa0_instance,
                                  false /* add_model_instance_prefix */,
                                  false /* always_add_suffix */);
  EXPECT_THAT(names, testing::ElementsAreArray({"iiwa_joint_1", "iiwa_joint_2",
                                                "iiwa_joint_3", "iiwa_joint_4",
                                                "iiwa_joint_5", "iiwa_joint_6",
                                                "iiwa_joint_7"}));

  names = plant->GetPositionNames(iiwa1_instance);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_link_0_qw", "iiwa_link_0_qx", "iiwa_link_0_qy",
                          "iiwa_link_0_qz", "iiwa_link_0_x", "iiwa_link_0_y",
                          "iiwa_link_0_z", "iiwa_joint_1_q", "iiwa_joint_2_q",
                          "iiwa_joint_3_q", "iiwa_joint_4_q", "iiwa_joint_5_q",
                          "iiwa_joint_6_q", "iiwa_joint_7_q"}));

  names = plant->GetPositionNames();
  EXPECT_THAT(
      names,
      testing::ElementsAreArray(
          {"iiwa0::iiwa14_iiwa_joint_1_q", "iiwa0::iiwa14_iiwa_joint_2_q",
           "iiwa0::iiwa14_iiwa_joint_3_q", "iiwa0::iiwa14_iiwa_joint_4_q",
           "iiwa0::iiwa14_iiwa_joint_5_q", "iiwa0::iiwa14_iiwa_joint_6_q",
           "iiwa0::iiwa14_iiwa_joint_7_q", "iiwa1::iiwa14_iiwa_link_0_qw",
           "iiwa1::iiwa14_iiwa_link_0_qx", "iiwa1::iiwa14_iiwa_link_0_qy",
           "iiwa1::iiwa14_iiwa_link_0_qz", "iiwa1::iiwa14_iiwa_link_0_x",
           "iiwa1::iiwa14_iiwa_link_0_y",  "iiwa1::iiwa14_iiwa_link_0_z",
           "iiwa1::iiwa14_iiwa_joint_1_q", "iiwa1::iiwa14_iiwa_joint_2_q",
           "iiwa1::iiwa14_iiwa_joint_3_q", "iiwa1::iiwa14_iiwa_joint_4_q",
           "iiwa1::iiwa14_iiwa_joint_5_q", "iiwa1::iiwa14_iiwa_joint_6_q",
           "iiwa1::iiwa14_iiwa_joint_7_q"}));

  names = plant->GetPositionNames(false /* add_model_instance_prefix */,
                                  false /* always_add_suffix */);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_joint_1",   "iiwa_joint_2",   "iiwa_joint_3",
                          "iiwa_joint_4",   "iiwa_joint_5",   "iiwa_joint_6",
                          "iiwa_joint_7",   "iiwa_link_0_qw", "iiwa_link_0_qx",
                          "iiwa_link_0_qy", "iiwa_link_0_qz", "iiwa_link_0_x",
                          "iiwa_link_0_y",  "iiwa_link_0_z",  "iiwa_joint_1",
                          "iiwa_joint_2",   "iiwa_joint_3",   "iiwa_joint_4",
                          "iiwa_joint_5",   "iiwa_joint_6",   "iiwa_joint_7"}));

  names = plant->GetVelocityNames(iiwa0_instance);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_joint_1_w", "iiwa_joint_2_w", "iiwa_joint_3_w",
                          "iiwa_joint_4_w", "iiwa_joint_5_w", "iiwa_joint_6_w",
                          "iiwa_joint_7_w"}));

  names = plant->GetVelocityNames(iiwa0_instance,
                                  true /* add_model_instance_prefix */,
                                  true /* always_add_suffix */);
  EXPECT_THAT(
      names,
      testing::ElementsAreArray(
          {"iiwa0::iiwa14_iiwa_joint_1_w", "iiwa0::iiwa14_iiwa_joint_2_w",
           "iiwa0::iiwa14_iiwa_joint_3_w", "iiwa0::iiwa14_iiwa_joint_4_w",
           "iiwa0::iiwa14_iiwa_joint_5_w", "iiwa0::iiwa14_iiwa_joint_6_w",
           "iiwa0::iiwa14_iiwa_joint_7_w"}));

  names = plant->GetVelocityNames(iiwa0_instance,
                                  true /* add_model_instance_prefix */,
                                  false /* always_add_suffix */);
  EXPECT_THAT(names,
              testing::ElementsAreArray(
                  {"iiwa0::iiwa14_iiwa_joint_1", "iiwa0::iiwa14_iiwa_joint_2",
                   "iiwa0::iiwa14_iiwa_joint_3", "iiwa0::iiwa14_iiwa_joint_4",
                   "iiwa0::iiwa14_iiwa_joint_5", "iiwa0::iiwa14_iiwa_joint_6",
                   "iiwa0::iiwa14_iiwa_joint_7"}));

  names = plant->GetVelocityNames(iiwa0_instance,
                                  false /* add_model_instance_prefix */,
                                  true /* always_add_suffix */);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_joint_1_w", "iiwa_joint_2_w", "iiwa_joint_3_w",
                          "iiwa_joint_4_w", "iiwa_joint_5_w", "iiwa_joint_6_w",
                          "iiwa_joint_7_w"}));

  names = plant->GetVelocityNames(iiwa0_instance,
                                  false /* add_model_instance_prefix */,
                                  false /* always_add_suffix */);
  EXPECT_THAT(names, testing::ElementsAreArray({"iiwa_joint_1", "iiwa_joint_2",
                                                "iiwa_joint_3", "iiwa_joint_4",
                                                "iiwa_joint_5", "iiwa_joint_6",
                                                "iiwa_joint_7"}));

  names = plant->GetVelocityNames(iiwa1_instance);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_link_0_wx", "iiwa_link_0_wy", "iiwa_link_0_wz",
                          "iiwa_link_0_vx", "iiwa_link_0_vy", "iiwa_link_0_vz",
                          "iiwa_joint_1_w", "iiwa_joint_2_w", "iiwa_joint_3_w",
                          "iiwa_joint_4_w", "iiwa_joint_5_w", "iiwa_joint_6_w",
                          "iiwa_joint_7_w"}));

  names = plant->GetVelocityNames();
  EXPECT_THAT(
      names,
      testing::ElementsAreArray(
          {"iiwa0::iiwa14_iiwa_joint_1_w", "iiwa0::iiwa14_iiwa_joint_2_w",
           "iiwa0::iiwa14_iiwa_joint_3_w", "iiwa0::iiwa14_iiwa_joint_4_w",
           "iiwa0::iiwa14_iiwa_joint_5_w", "iiwa0::iiwa14_iiwa_joint_6_w",
           "iiwa0::iiwa14_iiwa_joint_7_w", "iiwa1::iiwa14_iiwa_link_0_wx",
           "iiwa1::iiwa14_iiwa_link_0_wy", "iiwa1::iiwa14_iiwa_link_0_wz",
           "iiwa1::iiwa14_iiwa_link_0_vx", "iiwa1::iiwa14_iiwa_link_0_vy",
           "iiwa1::iiwa14_iiwa_link_0_vz", "iiwa1::iiwa14_iiwa_joint_1_w",
           "iiwa1::iiwa14_iiwa_joint_2_w", "iiwa1::iiwa14_iiwa_joint_3_w",
           "iiwa1::iiwa14_iiwa_joint_4_w", "iiwa1::iiwa14_iiwa_joint_5_w",
           "iiwa1::iiwa14_iiwa_joint_6_w", "iiwa1::iiwa14_iiwa_joint_7_w"}));

  names = plant->GetVelocityNames(false /* add_model_instance_prefix */,
                                  false /* always_add_suffix */);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_joint_1",   "iiwa_joint_2",   "iiwa_joint_3",
                          "iiwa_joint_4",   "iiwa_joint_5",   "iiwa_joint_6",
                          "iiwa_joint_7",   "iiwa_link_0_wx", "iiwa_link_0_wy",
                          "iiwa_link_0_wz", "iiwa_link_0_vx", "iiwa_link_0_vy",
                          "iiwa_link_0_vz", "iiwa_joint_1",   "iiwa_joint_2",
                          "iiwa_joint_3",   "iiwa_joint_4",   "iiwa_joint_5",
                          "iiwa_joint_6",   "iiwa_joint_7"}));

  names = plant->GetStateNames(iiwa0_instance);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_joint_1_q", "iiwa_joint_2_q", "iiwa_joint_3_q",
                          "iiwa_joint_4_q", "iiwa_joint_5_q", "iiwa_joint_6_q",
                          "iiwa_joint_7_q", "iiwa_joint_1_w", "iiwa_joint_2_w",
                          "iiwa_joint_3_w", "iiwa_joint_4_w", "iiwa_joint_5_w",
                          "iiwa_joint_6_w", "iiwa_joint_7_w"}));

  names = plant->GetStateNames(iiwa0_instance,
                               true /* add_model_instance_prefix */);
  EXPECT_THAT(
      names,
      testing::ElementsAreArray(
          {"iiwa0::iiwa14_iiwa_joint_1_q", "iiwa0::iiwa14_iiwa_joint_2_q",
           "iiwa0::iiwa14_iiwa_joint_3_q", "iiwa0::iiwa14_iiwa_joint_4_q",
           "iiwa0::iiwa14_iiwa_joint_5_q", "iiwa0::iiwa14_iiwa_joint_6_q",
           "iiwa0::iiwa14_iiwa_joint_7_q", "iiwa0::iiwa14_iiwa_joint_1_w",
           "iiwa0::iiwa14_iiwa_joint_2_w", "iiwa0::iiwa14_iiwa_joint_3_w",
           "iiwa0::iiwa14_iiwa_joint_4_w", "iiwa0::iiwa14_iiwa_joint_5_w",
           "iiwa0::iiwa14_iiwa_joint_6_w", "iiwa0::iiwa14_iiwa_joint_7_w"}));

  names = plant->GetStateNames();
  EXPECT_THAT(
      names,
      testing::ElementsAreArray(
          {"iiwa0::iiwa14_iiwa_joint_1_q", "iiwa0::iiwa14_iiwa_joint_2_q",
           "iiwa0::iiwa14_iiwa_joint_3_q", "iiwa0::iiwa14_iiwa_joint_4_q",
           "iiwa0::iiwa14_iiwa_joint_5_q", "iiwa0::iiwa14_iiwa_joint_6_q",
           "iiwa0::iiwa14_iiwa_joint_7_q", "iiwa1::iiwa14_iiwa_link_0_qw",
           "iiwa1::iiwa14_iiwa_link_0_qx", "iiwa1::iiwa14_iiwa_link_0_qy",
           "iiwa1::iiwa14_iiwa_link_0_qz", "iiwa1::iiwa14_iiwa_link_0_x",
           "iiwa1::iiwa14_iiwa_link_0_y",  "iiwa1::iiwa14_iiwa_link_0_z",
           "iiwa1::iiwa14_iiwa_joint_1_q", "iiwa1::iiwa14_iiwa_joint_2_q",
           "iiwa1::iiwa14_iiwa_joint_3_q", "iiwa1::iiwa14_iiwa_joint_4_q",
           "iiwa1::iiwa14_iiwa_joint_5_q", "iiwa1::iiwa14_iiwa_joint_6_q",
           "iiwa1::iiwa14_iiwa_joint_7_q", "iiwa0::iiwa14_iiwa_joint_1_w",
           "iiwa0::iiwa14_iiwa_joint_2_w", "iiwa0::iiwa14_iiwa_joint_3_w",
           "iiwa0::iiwa14_iiwa_joint_4_w", "iiwa0::iiwa14_iiwa_joint_5_w",
           "iiwa0::iiwa14_iiwa_joint_6_w", "iiwa0::iiwa14_iiwa_joint_7_w",
           "iiwa1::iiwa14_iiwa_link_0_wx", "iiwa1::iiwa14_iiwa_link_0_wy",
           "iiwa1::iiwa14_iiwa_link_0_wz", "iiwa1::iiwa14_iiwa_link_0_vx",
           "iiwa1::iiwa14_iiwa_link_0_vy", "iiwa1::iiwa14_iiwa_link_0_vz",
           "iiwa1::iiwa14_iiwa_joint_1_w", "iiwa1::iiwa14_iiwa_joint_2_w",
           "iiwa1::iiwa14_iiwa_joint_3_w", "iiwa1::iiwa14_iiwa_joint_4_w",
           "iiwa1::iiwa14_iiwa_joint_5_w", "iiwa1::iiwa14_iiwa_joint_6_w",
           "iiwa1::iiwa14_iiwa_joint_7_w"}));

  names = plant->GetStateNames(false /* add_model_instance_prefix */);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_joint_1_q", "iiwa_joint_2_q", "iiwa_joint_3_q",
                          "iiwa_joint_4_q", "iiwa_joint_5_q", "iiwa_joint_6_q",
                          "iiwa_joint_7_q", "iiwa_link_0_qw", "iiwa_link_0_qx",
                          "iiwa_link_0_qy", "iiwa_link_0_qz", "iiwa_link_0_x",
                          "iiwa_link_0_y",  "iiwa_link_0_z",  "iiwa_joint_1_q",
                          "iiwa_joint_2_q", "iiwa_joint_3_q", "iiwa_joint_4_q",
                          "iiwa_joint_5_q", "iiwa_joint_6_q", "iiwa_joint_7_q",
                          "iiwa_joint_1_w", "iiwa_joint_2_w", "iiwa_joint_3_w",
                          "iiwa_joint_4_w", "iiwa_joint_5_w", "iiwa_joint_6_w",
                          "iiwa_joint_7_w", "iiwa_link_0_wx", "iiwa_link_0_wy",
                          "iiwa_link_0_wz", "iiwa_link_0_vx", "iiwa_link_0_vy",
                          "iiwa_link_0_vz", "iiwa_joint_1_w", "iiwa_joint_2_w",
                          "iiwa_joint_3_w", "iiwa_joint_4_w", "iiwa_joint_5_w",
                          "iiwa_joint_6_w", "iiwa_joint_7_w"}));

  names = plant->GetActuatorNames(iiwa0_instance);
  EXPECT_THAT(names, testing::ElementsAreArray({"iiwa_joint_1", "iiwa_joint_2",
                                                "iiwa_joint_3", "iiwa_joint_4",
                                                "iiwa_joint_5", "iiwa_joint_6",
                                                "iiwa_joint_7"}));

  names = plant->GetActuatorNames(iiwa0_instance,
                                  true /* add_model_instance_prefix */);
  EXPECT_THAT(names,
              testing::ElementsAreArray(
                  {"iiwa0::iiwa14_iiwa_joint_1", "iiwa0::iiwa14_iiwa_joint_2",
                   "iiwa0::iiwa14_iiwa_joint_3", "iiwa0::iiwa14_iiwa_joint_4",
                   "iiwa0::iiwa14_iiwa_joint_5", "iiwa0::iiwa14_iiwa_joint_6",
                   "iiwa0::iiwa14_iiwa_joint_7"}));

  names = plant->GetActuatorNames();
  EXPECT_THAT(
      names, testing::ElementsAreArray(
                 {"iiwa0::iiwa14_iiwa_joint_1", "iiwa0::iiwa14_iiwa_joint_2",
                  "iiwa0::iiwa14_iiwa_joint_3", "iiwa0::iiwa14_iiwa_joint_4",
                  "iiwa0::iiwa14_iiwa_joint_5", "iiwa0::iiwa14_iiwa_joint_6",
                  "iiwa0::iiwa14_iiwa_joint_7", "iiwa1::iiwa14_iiwa_joint_1",
                  "iiwa1::iiwa14_iiwa_joint_2", "iiwa1::iiwa14_iiwa_joint_3",
                  "iiwa1::iiwa14_iiwa_joint_4", "iiwa1::iiwa14_iiwa_joint_5",
                  "iiwa1::iiwa14_iiwa_joint_6", "iiwa1::iiwa14_iiwa_joint_7"}));

  names = plant->GetActuatorNames(false /* add_model_instance_prefix */);
  EXPECT_THAT(names, testing::ElementsAreArray(
                         {"iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3",
                          "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6",
                          "iiwa_joint_7", "iiwa_joint_1", "iiwa_joint_2",
                          "iiwa_joint_3", "iiwa_joint_4", "iiwa_joint_5",
                          "iiwa_joint_6", "iiwa_joint_7"}));
}

GTEST_TEST(MultibodyPlantTest, FloatingJointNames) {
  {
    MultibodyPlant<double> plant(0.0);
    const ModelInstanceIndex my_model_instance =
        plant.AddModelInstance("MyModelInstance");

    // These bodies are in "my_model_instance" _not_ the default one. Their
    // body frames are in the same model instance.
    const RigidBody<double>& free_body = plant.AddRigidBody(
        "free_body", my_model_instance, SpatialInertia<double>::MakeUnitary());
    const RigidBody<double>& outer_body = plant.AddRigidBody(
        "outer_body", my_model_instance, SpatialInertia<double>::MakeUnitary());

    // But this frame is in the default model instance.
    auto frame_on_outer_ptr = std::make_unique<FixedOffsetFrame<double>>(
        "frame_on_outer", outer_body.body_frame(),
        RigidTransform<double>(Vector3<double>(1, 0, 0)),
        default_model_instance());
    const Frame<double>& frame_on_outer =
        plant.AddFrame(std::move(frame_on_outer_ptr));

    // A joint inherits its model instance from the child _frame_, not the
    // child _body_. Hence, this joint is in the default model instance while
    // the ephemeral floating joint will be in my_model_instance. This caused
    // a bug in the past (see issue #23379).
    auto the_joint_ptr = std::make_unique<RevoluteJoint<double>>(
        "the_joint", free_body.body_frame(), frame_on_outer,
        Vector3<double>(0, 0, 1));
    const Joint<double>& the_joint = plant.AddJoint(std::move(the_joint_ptr));

    // This weld joint is in "my_model_instance", but follows an unrelated
    // joint, so its "start coordinate" is outside the range of coordinates
    // in use by my_model_instance. That shouldn't cause any problems, but
    // did in the past (see issue #23379).
    const RigidBody<double>& last_body = plant.AddRigidBody(
        "last_body", my_model_instance, SpatialInertia<double>::MakeUnitary());
    const Joint<double>& weld_joint =
        plant.AddJoint<WeldJoint>("weld_joint", last_body, {}, outer_body, {},
                                  RigidTransform<double>::Identity());

    plant.Finalize();

    // Check that an ephemeral floating joint was added and named "free_body" to
    // match the body it mobilizes, and that it is in my_model_instance.
    const Joint<double>& floating_joint = plant.GetJointByName("free_body");
    EXPECT_EQ(floating_joint.num_positions(), 7);
    EXPECT_EQ(floating_joint.num_velocities(), 6);
    EXPECT_EQ(floating_joint.model_instance(), my_model_instance);

    // Check that the revolute joint followed frame_on_outer's model instance,
    // _not_ outer_body's. Its start coordinate follows the floating joint's 7.
    EXPECT_EQ(the_joint.model_instance(), default_model_instance());
    EXPECT_EQ(the_joint.position_start(), 7);
    EXPECT_EQ(the_joint.num_positions(), 1);

    // Check that the weld joint is in my_model_instance and that its start
    // coordinate is out of range for that model instance, which has only
    // 0-6 for the floating joint. The 7th is for the revolute joint (in the
    // default model instance). Hence the next available is 8.
    EXPECT_EQ(weld_joint.model_instance(), my_model_instance);
    EXPECT_EQ(weld_joint.position_start(), 8);
    EXPECT_EQ(weld_joint.num_positions(), 0);

    // There was a bug where either
    //   - the different model instances for child frame and child body, or
    //   - the out-of-range start coordinate for the weld joint
    // caused GetPositionNames(model_instance) to throw.
    // Check that the various forms all work now.
    EXPECT_NO_THROW(plant.GetPositionNames());
    EXPECT_NO_THROW(plant.GetPositionNames(my_model_instance));
    EXPECT_NO_THROW(plant.GetPositionNames(default_model_instance()));
  }

  // Verify that in case of a name conflict, we prepend with underscores
  // until the floating joint name is unique.
  {
    MultibodyPlant<double> plant(0.0);
    const RigidBody<double>& base_body =
        plant.AddRigidBody("base_body", default_model_instance(),
                           SpatialInertia<double>::MakeUnitary());
    const RigidBody<double>& body2 =
        plant.AddRigidBody("body2", default_model_instance(),
                           SpatialInertia<double>::MakeUnitary());
    const RigidBody<double>& body3 =
        plant.AddRigidBody("body3", default_model_instance(),
                           SpatialInertia<double>::MakeUnitary());

    // Add joints with very unfortunate names.
    plant.AddJoint<RevoluteJoint>("base_body", base_body, {}, body2, {},
                                  Vector3<double>(0, 0, 1));
    plant.AddJoint<RevoluteJoint>("_base_body", base_body, {}, body3, {},
                                  Vector3<double>(0, 0, 1));
    plant.Finalize();

    EXPECT_NO_THROW(plant.GetJointByName("__base_body"));
  }
}

GTEST_TEST(MultibodyPlantTest, GetMutableSceneGraphPreFinalize) {
  MultibodyPlant<double> plant(0.0);
  DRAKE_EXPECT_THROWS_MESSAGE(plant.GetMutableSceneGraphPreFinalize(),
                              ".*geometry_source_is_registered.*");

  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  EXPECT_EQ(plant.GetMutableSceneGraphPreFinalize(), &scene_graph);

  plant.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(plant.GetMutableSceneGraphPreFinalize(),
                              ".*!is_finalized.*");
}

GTEST_TEST(MultibodyPlantTest, RenameModelInstance) {
  // Much of the functionality is tested as part of *Tree. Here we test the
  // additional semantics of *Plant.
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.RenameModelInstance(ModelInstanceIndex(99), "invalid"),
      ".*no model instance id 99.*");

  // These renames are allowed; it is up to applications to decide if they are
  // a good idea in a given situation.
  EXPECT_NO_THROW(plant.RenameModelInstance(world_model_instance(), "Mars"));
  EXPECT_NO_THROW(plant.RenameModelInstance(default_model_instance(), "hmm"));

  std::string robot = R"""(
<robot name="a">
  <link name="b">
    <collision>
      <geometry>
        <sphere radius="0.25"/>
      </geometry>
    </collision>
  </link>
</robot>
)""";

  Parser parser(&plant);
  auto models = parser.AddModelsFromString(robot, "urdf");
  ASSERT_EQ(models.size(), 1);
  auto& body = plant.GetBodyByName("b");
  auto frame_id = plant.GetBodyFrameIdOrThrow(body.index());
  auto& inspector = scene_graph.model_inspector();
  ASSERT_EQ(inspector.GetName(frame_id), "a::b");
  auto geoms = inspector.GetGeometries(frame_id);
  ASSERT_EQ(geoms.size(), 1);
  ASSERT_EQ(inspector.GetName(geoms[0]), "a::Sphere");

  plant.RenameModelInstance(models[0], "zzz");
  // Renaming affects scoped geometry names.
  EXPECT_EQ(inspector.GetName(frame_id), "zzz::b");
  EXPECT_EQ(inspector.GetName(geoms[0]), "zzz::Sphere");
  // Renaming allows reload.
  EXPECT_NO_THROW(parser.AddModelsFromString(robot, "urdf"));

  // New names must be unique.
  DRAKE_EXPECT_THROWS_MESSAGE(plant.RenameModelInstance(models[0], "a"),
                              ".*must be unique.*");

  // Renaming will silently skip frames and geometries that don't match the
  // typical scoped-name pattern.

  // Oddly renamed frames can evade renaming.
  scene_graph.RenameFrame(frame_id, "something_random");
  plant.RenameModelInstance(models[0], "xoxo");
  EXPECT_EQ(inspector.GetName(frame_id), "something_random");
  EXPECT_EQ(inspector.GetName(geoms[0]), "xoxo::Sphere");

  // As can peculiarly renamed geometries.
  scene_graph.RenameGeometry(geoms[0], "anything_else");
  plant.RenameModelInstance(models[0], "bbbb");
  EXPECT_EQ(inspector.GetName(frame_id), "something_random");
  EXPECT_EQ(inspector.GetName(geoms[0]), "anything_else");

  plant.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(plant.RenameModelInstance(models[0], "too_late"),
                              ".*finalized.*");
}

// Rework this test on 2026-09-01 per TAMSI deprecation. The only test logic we
// still need is to check for a post-Finalize call to change the approximation.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Verify the proper coordination of discrete contact approximations with their
// corresponding solvers.
GTEST_TEST(MultibodyPlantTests, DiscreteContactApproximation) {
  MultibodyPlant<double> plant(0.01);

  auto set_approximation_and_check_solver =
      [&plant](DiscreteContactApproximation approximation) {
        plant.set_discrete_contact_approximation(approximation);
        EXPECT_EQ(plant.get_discrete_contact_approximation(), approximation);
        if (approximation == DiscreteContactApproximation::kTamsi) {
          // Only the TAMSI solver can be used with the TAMSI approximation.
          EXPECT_EQ(plant.get_discrete_contact_solver(),
                    DiscreteContactSolver::kTamsi);
        } else {
          // Approximations other than TAMSI use the SAP solver.
          EXPECT_EQ(plant.get_discrete_contact_solver(),
                    DiscreteContactSolver::kSap);
        }
      };

  // Verify that setting an apprximation sets the proper solver.
  // We reset to TAMSI every other approximation to make sure SAP approximations
  // are changing the solver back to SAP.
  set_approximation_and_check_solver(DiscreteContactApproximation::kTamsi);
  set_approximation_and_check_solver(DiscreteContactApproximation::kSap);

  set_approximation_and_check_solver(DiscreteContactApproximation::kTamsi);
  set_approximation_and_check_solver(DiscreteContactApproximation::kLagged);

  set_approximation_and_check_solver(DiscreteContactApproximation::kTamsi);
  set_approximation_and_check_solver(DiscreteContactApproximation::kSimilar);

  // Make sure we can go back to TAMSI after kSimilar.
  set_approximation_and_check_solver(DiscreteContactApproximation::kTamsi);

  // Post-finalize calls to set_discrete_contact_approximation() throws.
  plant.Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.set_discrete_contact_approximation(
          DiscreteContactApproximation::kTamsi),
      "Post-finalize calls to '.*' are not allowed; .*");
}
#pragma GCC diagnostic pop

INSTANTIATE_TEST_SUITE_P(ContinousAndDiscreteRemodeling,
                         MultibodyPlantRemodelingParam,
                         ::testing::Values(0.0, 0.1));

INSTANTIATE_TEST_SUITE_P(ContinousAndDiscreteConstraints,
                         MultibodyPlantConstraintTestTimeStepParam,
                         ::testing::Values(0.0, 0.1));

}  // namespace
}  // namespace multibody
}  // namespace drake
