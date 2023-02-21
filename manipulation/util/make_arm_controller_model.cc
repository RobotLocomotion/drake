#include "drake/manipulation/util/make_arm_controller_model.h"

#include <string>
#include <vector>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/scoped_names.h"

namespace drake {
namespace manipulation {
namespace internal {

using math::RigidTransform;
using multibody::Body;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::parsing::GetScopedFrameName;
using multibody::parsing::ModelInstanceInfo;
using systems::Context;

namespace {

bool AreFramesWelded(const MultibodyPlant<double>& plant,
                     const Frame<double>& A, const Frame<double>& B) {
  if (&A.body() == &B.body())
    return true;
  for (const auto* body : plant.GetBodiesWeldedTo(A.body())) {
    if (body == &B.body())
      return true;
  }
  return false;
}

}  // namespace

std::unique_ptr<MultibodyPlant<double>> MakeArmControllerModel(
    const MultibodyPlant<double>& simulation_plant,
    const ModelInstanceInfo& arm_info,
    const std::optional<ModelInstanceInfo>& gripper_info) {
  log()->debug("MakeArmControllerModel:");
  log()->debug("  arm:");
  log()->debug("    model: {}", arm_info.model_path);
  log()->debug("    child_frame: {}", arm_info.child_frame_name);

  const ModelInstanceIndex sim_arm_model_index =
      simulation_plant.GetModelInstanceByName(arm_info.model_name);

  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  Parser parser(plant.get());
  const auto models = parser.AddModels(arm_info.model_path);
  DRAKE_DEMAND(models.size() == 1);
  const ModelInstanceIndex arm_model_index = models[0];
  // The arm must be anchored to the world.
  const Frame<double>& sim_arm_child_frame = simulation_plant.GetFrameByName(
      arm_info.child_frame_name, sim_arm_model_index);
  DRAKE_THROW_UNLESS(simulation_plant.IsAnchored(sim_arm_child_frame.body()));
  // Make sure that the arm is attached to the same location in the controller
  // plant as it is in the simulation plant.
  // TODO(siyuan): mbp should provide a way to query anchored frame kinematics
  // without a Context.
  std::unique_ptr<Context<double>> sim_context =
      simulation_plant.CreateDefaultContext();
  const RigidTransform<double> arm_X_WC =
      simulation_plant.CalcRelativeTransform(
          *sim_context, simulation_plant.world_frame(), sim_arm_child_frame);

  plant->WeldFrames(
      plant->world_frame(),
      plant->GetFrameByName(arm_info.child_frame_name, arm_model_index),
      arm_X_WC);

  // Get all models that are gripper-related. This includes all of the bodies
  // that are part of the gripper's model instance and anything welded to the
  // gripper (for example, the calibration checkerboard).
  if (gripper_info) {
    log()->debug("  gripper:");
    log()->debug("    model: {}", gripper_info->model_path);
    log()->debug("    parent_frame: {}", gripper_info->parent_frame_name);
    log()->debug("    child_frame: {}", gripper_info->child_frame_name);

    DRAKE_DEMAND(gripper_info->model_instance !=
                 multibody::default_model_instance());
    std::vector<BodyIndex> sim_gripper_body_indices =
        simulation_plant.GetBodyIndices(gripper_info->model_instance);

    const std::string gripper_body_name = gripper_info->child_frame_name;
    const Frame<double>& G = simulation_plant.GetFrameByName(
        gripper_body_name, gripper_info->model_instance);

    // TODO(sam-creasey) We actually want all kinematic children from the
    // gripper body, but there doesn't seem to be an existing way to do that
    // from MultibodyPlant.  MultibodyTreeTopology/BodyTopology have the needed
    // information, but it doesn't seem to be accessible.
    std::vector<const Body<double>*> sim_gripper_welded_bodies =
        simulation_plant.GetBodiesWeldedTo(G.body());

    for (const Body<double>* welded_body : sim_gripper_welded_bodies) {
      if ((welded_body->model_instance() != gripper_info->model_instance) &&
          (welded_body->model_instance() != arm_info.model_instance)) {
        log()->debug("Adding BodyIndex of Body {}", welded_body->name());
        sim_gripper_body_indices.push_back(welded_body->index());
      }
    }

    // `C` is the composite body for the gripper.
    const SpatialInertia<double> M_CGo_G = simulation_plant.CalcSpatialInertia(
        *sim_context, G, sim_gripper_body_indices);

    // Add surrogate composite body.
    const RigidBody<double>& C =
        plant->AddRigidBody(gripper_body_name, arm_model_index, M_CGo_G);

    // Make sure that the gripper is attached to the same location (and arm
    // frame) in the controller plant as it is in the simulation plant.
    const Frame<double>& sim_gripper_parent_frame =
        simulation_plant.GetFrameByName(gripper_info->parent_frame_name,
                                        arm_info.model_instance);
    // `Pp` is the "grand-parent" frame.
    const std::string gripper_grand_parent_frame_name =
        sim_gripper_parent_frame.body().body_frame().name();
    const Frame<double>& gripper_grand_parent_frame =
        plant->GetFrameByName(gripper_grand_parent_frame_name, arm_model_index);
    log()->trace("    gripper_grand_parent_frame: {}",
                 gripper_grand_parent_frame.scoped_name());
    const RigidTransform<double> gripper_X_PpP =
        sim_gripper_parent_frame.GetFixedPoseInBodyFrame();
    const RigidTransform<double> gripper_X_PpC =
        gripper_X_PpP * gripper_info->X_PC;
    plant->WeldFrames(gripper_grand_parent_frame, C.body_frame(),
                      gripper_X_PpC);

    // Add the grasp frame, F, to the plant if present.
    if (simulation_plant.HasFrameNamed("grasp_frame",
                                       gripper_info->model_instance)) {
      const Frame<double>& frame_F = simulation_plant.GetFrameByName(
          "grasp_frame", gripper_info->model_instance);
      DRAKE_DEMAND(AreFramesWelded(simulation_plant, frame_F, G));
      // Since F and G are rigidly attached, we can use the default context
      // to compute the offset.
      const RigidTransform<double> X_GF = frame_F.CalcPose(*sim_context, G);
      // Since the body frame of C in `plant` corresponds to frame G in
      // `simulation_plant`, the relative transform X_CF' (where F' is the grasp
      // frame in `plant`) should be equal to X_GF.
      plant->AddFrame(std::make_unique<multibody::FixedOffsetFrame<double>>(
          "grasp_frame", C.body_frame(), X_GF));
    }
  }

  plant->Finalize();
  return plant;
}

}  // namespace internal
}  // namespace manipulation
}  // namespace drake
