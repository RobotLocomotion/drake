#include "drake/manipulation/util/make_arm_controller_model.h"

#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/shared_pointer_system.h"

namespace drake {
namespace manipulation {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RollPitchYaw;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::PackageMap;
using multibody::Parser;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::parsing::LoadModelDirectives;
using multibody::parsing::ModelDirectives;
using multibody::parsing::ModelInstanceInfo;
using systems::Context;
using systems::DiagramBuilder;
using systems::SharedPointerSystem;

constexpr double kTolerance = 1e-9;

class MakeArmControllerModelTest : public ::testing::Test {
 public:
  MakeArmControllerModelTest()
      : builder_{},
        sim_plant_{builder_.AddSystem<MultibodyPlant<double>>(0.001)},
        iiwa7_model_path_(PackageMap{}.ResolveUrl(
            "package://drake_models/iiwa_description/sdf/"
            "iiwa7_no_collision.sdf")),
        wsg_model_path_(PackageMap{}.ResolveUrl(
            "package://drake_models/wsg_50_description/sdf/"
            "schunk_wsg_50.sdf")) {
    // Add an `empty` model instance to ensure model instance lookups are
    // correct between this plant and the newly constructed control plant.
    sim_plant_->AddModelInstance("empty");
  }

 protected:
  // Adds an Iiwa model into `sim_plant_` and returns its ModelInstanceInfo.
  ModelInstanceInfo AddIiwaModel() {
    const ModelInstanceIndex iiwa7_instance =
        Parser(sim_plant_).AddModels(iiwa7_model_path_).at(0);
    return {.model_name = sim_plant_->GetModelInstanceName(iiwa7_instance),
            .model_path = iiwa7_model_path_,
            .child_frame_name = "iiwa_link_0",
            .model_instance = iiwa7_instance};
  }

  // Adds a Wsg model into `sim_plant_` and returns its ModelInstanceInfo.
  ModelInstanceInfo AddWsgModel() {
    const ModelInstanceIndex wsg_instance =
        Parser(sim_plant_).AddModels(wsg_model_path_).at(0);
    return {.model_name = sim_plant_->GetModelInstanceName(wsg_instance),
            .model_path = wsg_model_path_,
            .child_frame_name = "body",
            .model_instance = wsg_instance};
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* sim_plant_;
  const std::string iiwa7_model_path_;
  const std::string wsg_model_path_;
};

// Returns all body indices in the `plant` except the world body index.
std::vector<BodyIndex> GetAllBodyIndices(const MultibodyPlant<double>& plant) {
  // The world body is index zero, and thus the loop iterates from index one.
  std::vector<BodyIndex> body_indices(plant.num_bodies() - 1);
  std::iota(body_indices.begin(), body_indices.end(), BodyIndex(1));
  return body_indices;
}

// This test blindly operates on the full simulation and control plants.
// Therefore, the simulation plant's mass should be wholly contained in bodies
// that belong to either the arm or (optional) gripper model instances, or be
// welded to a body in those instances.
void CompareInertialTerms(const MultibodyPlant<double>& sim_plant,
                          ModelInstanceIndex sim_arm_model_instance,
                          const MultibodyPlant<double>& control_plant,
                          const Context<double>& sim_plant_context) {
  // Create analogous control context.
  std::unique_ptr<Context<double>> control_plant_context =
      control_plant.CreateDefaultContext();
  const Eigen::VectorXd q_arm =
      sim_plant.GetPositions(sim_plant_context, sim_arm_model_instance);
  control_plant.SetPositions(control_plant_context.get(), q_arm);

  // Compare total mass.
  EXPECT_NEAR(sim_plant.CalcTotalMass(sim_plant_context),
              control_plant.CalcTotalMass(*control_plant_context), kTolerance);

  // Check the spatial inertial of the two plants.
  const SpatialInertia<double> M_CIo_W_sim = sim_plant.CalcSpatialInertia(
      sim_plant_context, sim_plant.world_frame(), GetAllBodyIndices(sim_plant));
  const SpatialInertia<double> M_CIo_W_control =
      control_plant.CalcSpatialInertia(*control_plant_context,
                                       control_plant.world_frame(),
                                       GetAllBodyIndices(control_plant));
  EXPECT_TRUE(CompareMatrices(M_CIo_W_sim.CopyToFullMatrix6(),
                              M_CIo_W_control.CopyToFullMatrix6(), kTolerance));

  const Eigen::VectorXd sim_gravity_full =
      sim_plant.CalcGravityGeneralizedForces(sim_plant_context);
  const Eigen::VectorXd sim_gravity =
      sim_plant.GetActuationFromArray(sim_arm_model_instance, sim_gravity_full);
  const Eigen::VectorXd control_gravity =
      control_plant.CalcGravityGeneralizedForces(*control_plant_context);
  EXPECT_TRUE(CompareMatrices(sim_gravity, control_gravity, kTolerance));
}

/* Creates a simple simulation MultibodyPlant with only one Iiwa arm, and the
 corresponding MultibodyPlant for control. The two plants should share the same
 properties, e.g., the number of models and actuation. */
TEST_F(MakeArmControllerModelTest, SingleIiwaWithoutWsg) {
  // Manually add an Iiwa arm to `sim_plant_` MultibodyPlant and weld it to the
  // world frame.
  const ModelInstanceInfo iiwa7_info = AddIiwaModel();
  // Create an arbitrary transform for testing.
  const RigidTransform<double> X_WIiwa_sim(
      RollPitchYaw<double>(0.0, 0.0, M_PI / 2),
      Eigen::Vector3d(1.0, -1.0, 0.0));
  sim_plant_->WeldFrames(sim_plant_->world_frame(),
                         sim_plant_->GetFrameByName(iiwa7_info.child_frame_name,
                                                    iiwa7_info.model_instance),
                         X_WIiwa_sim);
  sim_plant_->Finalize();
  ASSERT_GT(sim_plant_->num_multibody_states(), 0);

  MultibodyPlant<double>* control_plant =
      SharedPointerSystem<double>::AddToBuilder(
          &builder_, MakeArmControllerModel(*sim_plant_, iiwa7_info));
  ASSERT_NE(control_plant, nullptr);
  const auto diagram = builder_.Build();

  std::unique_ptr<Context<double>> sim_plant_context =
      sim_plant_->CreateDefaultContext();
  std::unique_ptr<Context<double>> control_plant_context =
      control_plant->CreateDefaultContext();

  // Both plants should have 14 states, i.e., [q, v] for the 7-DoF Iiwa.
  EXPECT_EQ(control_plant->num_multibody_states(),
            sim_plant_->num_multibody_states());
  // MultibodyPlant always creates at least two model instances, one for the
  // world and one for a default model instance for unspecified modeling
  // elements. Note that we add an additional model instance ("empty") to the
  // simulation plant to have meaningfully different model instance indices
  // between the two plants.
  EXPECT_EQ(control_plant->num_model_instances() + 1,
            sim_plant_->num_model_instances());
  EXPECT_EQ(control_plant->num_actuators(), sim_plant_->num_actuators());

  // Check the arm is welded to the world at the same pose.
  const Frame<double>& iiwa7_child_frame =
      control_plant->GetFrameByName(iiwa7_info.child_frame_name);
  EXPECT_TRUE(control_plant->IsAnchored(iiwa7_child_frame.body()));
  const RigidTransform<double> X_WIiwa_control =
      control_plant->CalcRelativeTransform(*control_plant_context,
                                           control_plant->world_frame(),
                                           iiwa7_child_frame);
  EXPECT_TRUE(X_WIiwa_control.IsExactlyEqualTo(X_WIiwa_sim));

  // `grasp_frame` should not present in `control_plant`.
  EXPECT_FALSE(control_plant->HasFrameNamed("grasp_frame"));

  // Check inertial terms.
  CompareInertialTerms(*sim_plant_, iiwa7_info.model_instance, *control_plant,
                       *sim_plant_context);
}

TEST_F(MakeArmControllerModelTest, DifferentGravity) {
  // Manually add an Iiwa arm to `sim_plant_` MultibodyPlant and weld it to the
  // world frame.
  const ModelInstanceInfo iiwa7_info = AddIiwaModel();
  sim_plant_->WeldFrames(sim_plant_->world_frame(),
                         sim_plant_->GetFrameByName(iiwa7_info.child_frame_name,
                                                    iiwa7_info.model_instance),
                         RigidTransform<double>());
  sim_plant_->mutable_gravity_field().set_gravity_vector(
      Eigen::Vector3d(0.1, 0.2, -0.3));
  sim_plant_->Finalize();

  std::unique_ptr<MultibodyPlant<double>> control_plant =
      MakeArmControllerModel(*sim_plant_, iiwa7_info);
  ASSERT_NE(control_plant, nullptr);

  std::unique_ptr<Context<double>> sim_plant_context =
      sim_plant_->CreateDefaultContext();

  CompareInertialTerms(*sim_plant_, iiwa7_info.model_instance, *control_plant,
                       *sim_plant_context);
}

/* Creates a more complex simulation MultibodyPlant from a directives file
 (contains an Iiwa arm, a Wsg gripper, and other models), and the corresponding
 MultibodyPlant for control. As MakeArmControllerModel() replaces the gripper
 model (if provided) with a RigidBody attaching to the arm, the mass and dynamic
 properties should be preserved but some of the plant-specific properties should
 not. */
TEST_F(MakeArmControllerModelTest, LoadIiwaWsgFromDirectives) {
  const ModelDirectives directives = LoadModelDirectives(
      FindResourceOrThrow("drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"));
  Parser parser{sim_plant_};
  const std::vector<ModelInstanceInfo> models_from_directives =
      multibody::parsing::ProcessModelDirectives(directives, &parser);

  sim_plant_->Finalize();
  ASSERT_GT(sim_plant_->num_multibody_states(), 0);

  // Query the ModelInstanceInfo(s) from the directives.
  ModelInstanceInfo iiwa7_info;
  ModelInstanceInfo wsg_info;
  for (const ModelInstanceInfo& model_info : models_from_directives) {
    if (model_info.model_name == "iiwa7") {
      iiwa7_info = model_info;
    } else if (model_info.model_name == "schunk_wsg") {
      wsg_info = model_info;
    }
  }
  // In this test case, `grasp_frame` is assumed to exist.
  ASSERT_TRUE(
      sim_plant_->HasFrameNamed("grasp_frame", wsg_info.model_instance));

  MultibodyPlant<double>* control_plant =
      SharedPointerSystem<double>::AddToBuilder(
          &builder_, MakeArmControllerModel(*sim_plant_, iiwa7_info, wsg_info));
  ASSERT_NE(control_plant, nullptr);
  const auto diagram = builder_.Build();

  std::unique_ptr<Context<double>> sim_plant_context =
      sim_plant_->CreateDefaultContext();
  std::unique_ptr<Context<double>> control_plant_context =
      control_plant->CreateDefaultContext();

  // The number of states for `control_plant` and `sim_plant_` should be
  // different. However, the number of states of the Iiwa arm instance should
  // be identical.
  EXPECT_NE(control_plant->num_multibody_states(),
            sim_plant_->num_multibody_states());
  EXPECT_EQ(control_plant->num_multibody_states(),
            sim_plant_->num_multibody_states(iiwa7_info.model_instance));
  EXPECT_NE(control_plant->num_model_instances(),
            sim_plant_->num_model_instances());
  // `sim_plant_` should have 7 (Iiwa) + 2 (Wsg) actuations, while
  // `control_plant` should have only 7. However, the number of actuated DoFs
  // for the Iiwa arm should be identical.
  EXPECT_NE(control_plant->num_actuators(), sim_plant_->num_actuators());
  EXPECT_EQ(control_plant->num_actuated_dofs(),
            sim_plant_->num_actuated_dofs(iiwa7_info.model_instance));

  const Frame<double>& sim_iiwa7_child_frame = sim_plant_->GetFrameByName(
      iiwa7_info.child_frame_name, iiwa7_info.model_instance);
  const Frame<double>& control_iiwa7_child_frame =
      control_plant->GetFrameByName(iiwa7_info.child_frame_name);

  // Check the arm is welded to the world at the same pose.
  const RigidTransform<double> X_WIiwa_sim = sim_plant_->CalcRelativeTransform(
      *sim_plant_context, sim_plant_->world_frame(), sim_iiwa7_child_frame);
  const RigidTransform<double> X_WIiwa_control =
      control_plant->CalcRelativeTransform(*control_plant_context,
                                           control_plant->world_frame(),
                                           control_iiwa7_child_frame);
  EXPECT_TRUE(control_plant->IsAnchored(control_iiwa7_child_frame.body()));
  EXPECT_TRUE(X_WIiwa_sim.IsExactlyEqualTo(X_WIiwa_control));

  // Check `grasp_frame` is added at the same pose as `sim_plant_`.
  EXPECT_TRUE(control_plant->HasFrameNamed("grasp_frame"));
  const Frame<double>& sim_wsg_child_frame = sim_plant_->GetFrameByName(
      wsg_info.child_frame_name, wsg_info.model_instance);
  const Frame<double>& sim_wsg_grasp_frame =
      sim_plant_->GetFrameByName("grasp_frame", wsg_info.model_instance);
  const RigidTransform<double> X_GF_sim =
      sim_wsg_grasp_frame.CalcPose(*sim_plant_context, sim_wsg_child_frame);

  const Frame<double>& control_wsg_child_frame =
      control_plant->GetFrameByName(wsg_info.child_frame_name);
  const Frame<double>& control_wsg_grasp_frame =
      control_plant->GetFrameByName("grasp_frame");
  const RigidTransform<double> X_GF_control = control_wsg_grasp_frame.CalcPose(
      *control_plant_context, control_wsg_child_frame);
  EXPECT_TRUE(X_GF_sim.IsNearlyEqualTo(X_GF_control, kTolerance));

  CompareInertialTerms(*sim_plant_, iiwa7_info.model_instance, *control_plant,
                       *sim_plant_context);
}

TEST_F(MakeArmControllerModelTest, AdditionalAttachedModels) {
  const ModelDirectives directives = LoadModelDirectives(FindResourceOrThrow(
      "drake/manipulation/util/test/iiwa7_wsg_cameras.dmd.yaml"));
  Parser parser{sim_plant_};
  const std::vector<ModelInstanceInfo> models_from_directives =
      multibody::parsing::ProcessModelDirectives(directives, &parser);
  sim_plant_->Finalize();

  // Query the ModelInstanceInfo(s) from the directives.
  ModelInstanceInfo iiwa7_info;
  ModelInstanceInfo wsg_info;
  for (const ModelInstanceInfo& model_info : models_from_directives) {
    if (model_info.model_name == "iiwa7") {
      iiwa7_info = model_info;
    } else if (model_info.model_name == "schunk_wsg") {
      wsg_info = model_info;
    }
  }

  std::unique_ptr<MultibodyPlant<double>> control_plant =
      MakeArmControllerModel(*sim_plant_, iiwa7_info, wsg_info);
  ASSERT_NE(control_plant, nullptr);

  std::unique_ptr<Context<double>> sim_plant_context =
      sim_plant_->CreateDefaultContext();

  CompareInertialTerms(*sim_plant_, iiwa7_info.model_instance, *control_plant,
                       *sim_plant_context);
}

}  // namespace
}  // namespace internal
}  // namespace manipulation
}  // namespace drake
