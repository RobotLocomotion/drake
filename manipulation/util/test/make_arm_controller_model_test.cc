#include "sim/common/make_arm_controller_model.h"

#include <memory>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/shared_pointer_system.h"
#include "common/find_resource.h"

using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using drake::multibody::Frame;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::multibody::parsing::LoadModelDirectives;
using drake::multibody::parsing::ModelDirectives;
using drake::multibody::parsing::ModelInstanceInfo;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::SharedPointerSystem;
using Eigen::Vector3d;

namespace anzu {
namespace sim {
namespace internal {
namespace {

constexpr double kTolerance = 1e-9;

class MakeArmControllerModelTest : public ::testing::Test {
 public:
  MakeArmControllerModelTest()
      : builder_{},
        sim_plant_{builder_.AddSystem<MultibodyPlant<double>>(0.001)},
        iiwa7_model_path_(drake::FindResourceOrThrow(
            "drake/manipulation/models/iiwa_description/iiwa7"
            "/iiwa7_no_collision.sdf")),
        wsg_model_path_(drake::FindResourceOrThrow(
            "drake/manipulation/models/wsg_50_description/sdf/"
            "schunk_wsg_50.sdf")) {}

 protected:
  // Adds an Iiwa model into `sim_plant_` and returns its ModelInstanceInfo.
  ModelInstanceInfo AddIiwaModel(const std::string& iiwa7_model_name) {
    const ModelInstanceIndex iiwa7_instance =
        Parser(sim_plant_)
            .AddModelFromFile(iiwa7_model_path_, iiwa7_model_name);
    return {.model_name = iiwa7_model_name,
            .model_path = iiwa7_model_path_,
            .child_frame_name = "iiwa_link_0",
            .model_instance = iiwa7_instance};
  }

  // Adds a Wsg model into `sim_plant_` and returns its ModelInstanceInfo.
  ModelInstanceInfo AddWsgModel(const std::string& wsg_model_name) {
    const ModelInstanceIndex wsg_instance =
        Parser(sim_plant_).AddModelFromFile(wsg_model_path_, wsg_model_name);
    return {.model_name = wsg_model_name,
            .model_path = wsg_model_path_,
            .child_frame_name = "body",
            .model_instance = wsg_instance};
  }

  DiagramBuilder<double> builder_;
  MultibodyPlant<double>* sim_plant_;
  const std::string iiwa7_model_path_;
  const std::string wsg_model_path_;
};

// A helper funtion that uses plant.GetBodyIndices() internally to aggregate
// `BodyIndex`s across multiple `ModelInstanceIndex`s.
std::vector<BodyIndex> GetBodyIndices(
    const MultibodyPlant<double>& plant,
    const std::vector<ModelInstanceIndex>& models) {
  std::vector<BodyIndex> body_indices;
  for (const auto model : models) {
    const std::vector<BodyIndex> indices = plant.GetBodyIndices(model);
    body_indices.insert(body_indices.end(), indices.begin(), indices.end());
  }
  return body_indices;
}

// Returns all body indices in the `plant` except the world body index.
std::vector<BodyIndex> GetAllBodyIndices(const MultibodyPlant<double>& plant) {
  // The world body is index zero, and thus the loop iterates from index one.
  std::vector<BodyIndex> body_indices(plant.num_bodies() - 1);
  std::iota(body_indices.begin(), body_indices.end(), BodyIndex(1));
  return body_indices;
}

TEST_F(MakeArmControllerModelTest, GetBodyIndices) {
  // An empty `models` should have zero bodies.
  EXPECT_EQ(GetBodyIndices(*sim_plant_, {}).size(), 0);

  const ModelInstanceInfo iiwa7_info = AddIiwaModel("iiwa7_model");
  EXPECT_EQ(GetBodyIndices(*sim_plant_, {iiwa7_info.model_instance}).size(), 8);
  // Query an empty `models` in a non-empty plant should still return zero.
  EXPECT_EQ(GetBodyIndices(*sim_plant_, {}).size(), 0);

  const ModelInstanceInfo wsg_info = AddWsgModel("wsg_model");
  EXPECT_EQ(GetBodyIndices(*sim_plant_, {wsg_info.model_instance}).size(), 3);

  const std::vector<ModelInstanceIndex> wsg_iiwa7{wsg_info.model_instance,
                                                  iiwa7_info.model_instance};
  EXPECT_EQ(GetBodyIndices(*sim_plant_, wsg_iiwa7).size(), 11);
}

/* Creates a simple simulation MultibodyPlant with only one Iiwa arm, and the
 corresponding MultibodyPlant for control. The two plants should share the same
 properties, e.g., the number of models and actuation. */
TEST_F(MakeArmControllerModelTest, SingleIiwaWithoutWsg) {
  // Manually add an Iiwa arm to `sim_plant_` MultibodyPlant and weld it to the
  // world frame.
  const ModelInstanceInfo iiwa7_info = AddIiwaModel("iiwa7_model");
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
  // elements. The third one is the added Iiwa model.
  EXPECT_EQ(control_plant->num_model_instances(),
            sim_plant_->num_model_instances());
  EXPECT_EQ(control_plant->num_actuators(), sim_plant_->num_actuators());

  // Check the arm is welded to the world at the same pose.
  const Frame<double>& iiwa7_child_frame = control_plant->GetFrameByName(
      iiwa7_info.child_frame_name, iiwa7_info.model_instance);
  EXPECT_TRUE(control_plant->IsAnchored(iiwa7_child_frame.body()));
  const RigidTransform<double> X_WIiwa_control =
      control_plant->CalcRelativeTransform(*control_plant_context,
                                           control_plant->world_frame(),
                                           iiwa7_child_frame);
  EXPECT_TRUE(X_WIiwa_control.IsExactlyEqualTo(X_WIiwa_sim));

  // Check the mass property.
  EXPECT_EQ(sim_plant_->CalcTotalMass(*sim_plant_context),
            control_plant->CalcTotalMass(*control_plant_context));

  // `grasp_frame` should not present in `control_plant`.
  EXPECT_FALSE(control_plant->HasFrameNamed("grasp_frame"));
}

/* Creates a more complex simulation MultibodyPlant from a directives file
 (contains an Iiwa arm, a Wsg gripper, and other models), and the corresponding
 MultibodyPlant for control. As MakeArmControllerModel() replaces the gripper
 model (if provided) with a RigidBody attaching to the arm, the mass and dynamic
 properties should be preserved but some of the plant-specific properties should
 not. */
TEST_F(MakeArmControllerModelTest, LoadIiwaWsgFromDirectives) {
  const ModelDirectives directives = LoadModelDirectives(
      FindAnzuResourceOrThrow("sim/common/test/iiwa7_wsg.dmd.yaml"));
  Parser parser{sim_plant_};
  std::vector<ModelInstanceInfo> models_from_directives =
      drake::multibody::parsing::ProcessModelDirectives(directives, &parser);

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

  // Check the total mass of the Iiwa arm and the Wsg gripper.
  const std::vector<ModelInstanceIndex> sim_plant_iiwa7_wsg = {
      iiwa7_info.model_instance, wsg_info.model_instance};
  EXPECT_EQ(sim_plant_->CalcTotalMass(*sim_plant_context, sim_plant_iiwa7_wsg),
            control_plant->CalcTotalMass(*control_plant_context));

  // Check the spatial inertial matrix of the Iiwa arm and the Wsg gripper.
  const SpatialInertia<double> M_CIo_I_sim = sim_plant_->CalcSpatialInertia(
      *sim_plant_context, sim_iiwa7_child_frame,
      GetBodyIndices(*sim_plant_, sim_plant_iiwa7_wsg));
  const SpatialInertia<double> M_CIo_I_control =
      control_plant->CalcSpatialInertia(*control_plant_context,
                                        control_iiwa7_child_frame,
                                        GetAllBodyIndices(*control_plant));
  EXPECT_TRUE(drake::CompareMatrices(M_CIo_I_sim.CopyToFullMatrix6(),
                                     M_CIo_I_control.CopyToFullMatrix6(),
                                     kTolerance));

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
}

}  // namespace
}  // namespace internal
}  // namespace sim
}  // namespace anzu
