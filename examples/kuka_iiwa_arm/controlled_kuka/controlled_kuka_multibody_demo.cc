/// @file
///
/// This demo sets up a position controlled and gravity compensated KUKA iiwa
/// robot within a MultibodyPlant simulation to follow an arbitrarily designed
/// plan. The generated plan takes the arm from the zero configuration to reach
/// to a position in space and then repeat this reaching task with a different
/// joint configuration constraint.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/examples/kuka_iiwa_arm/controlled_kuka/controlled_kuka_trajectory.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"

DEFINE_double(simulation_sec, 0.1, "Number of seconds to simulate.");

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::math::RollPitchYaw;
using drake::multibody::Body;
using drake::multibody::ModelInstanceIndex ;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::MultibodyTree;
using drake::multibody::parsing::AddModelFromSdfFile;
using drake::multibody::Joint;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
using trajectories::PiecewisePolynomial;

const char kSdfPath[] =
    "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf";

const char kWsg50SdfPath[] =
    "drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50.sdf";

SpatialInertia<double> MakeCompositeGripperInertia() {
  MultibodyPlant<double> plant;
  AddModelFromSdfFile(FindResourceOrThrow(kWsg50SdfPath), &plant);
  plant.Finalize();

  const auto& gripper_body = plant.tree().GetRigidBodyByName("body");
  const auto& left_finger = plant.tree().GetRigidBodyByName("left_finger");
  const auto& right_finger = plant.tree().GetRigidBodyByName("right_finger");

  const auto& left_slider = plant.GetJointByName("left_finger_sliding_joint");
  const auto& right_slider = plant.GetJointByName("right_finger_sliding_joint");

  const SpatialInertia<double>& M_GGo_G = gripper_body.default_spatial_inertia();
  const SpatialInertia<double>& M_LLo_L =
      left_finger.default_spatial_inertia();
  const SpatialInertia<double>& M_RRo_R =
      right_finger.default_spatial_inertia();

  auto CalcFingerPoseInGripperFrame = [](const Joint<double>& slider) {
    // Pose of the joint's parent frame P (attached on gripper body G) in the
    // frame of the gripper G.
    const Isometry3<double> X_GP =
        slider.frame_on_parent().GetFixedPoseInBodyFrame();
    // Pose of the joint's child frame C (attached on the slider's finger body)
    // in the frame of the slider's finger F.
    const Isometry3<double> X_FC =
        slider.frame_on_child().GetFixedPoseInBodyFrame();
    // When the slider's translational dof is zero, then P coincides with C.
    // Therefore:
    const Isometry3<double> X_GF = X_GP * X_FC.inverse();
    return X_GF;
  };

  // Pose of the left finger L in the gripper frame G when the slider's dof is
  // zero.
  const Isometry3<double> X_GL = CalcFingerPoseInGripperFrame(left_slider);

  // Pose of the right finger R in the gripper frame G when the slider's dof is
  // zero.
  const Isometry3<double> X_GR = CalcFingerPoseInGripperFrame(right_slider);

  // Helper to compute the spatial inertia of a finger F in about the gripper's
  // origin Go, expressed in G.
  auto CalcFingerSpatialInertiaInGripperFrame = [](
      const SpatialInertia<double>& M_FFo_F, const Isometry3<double>& X_GF) {
    const auto M_FFo_G = M_FFo_F.ReExpress(X_GF.linear());
    const auto p_FoGo_G = -X_GF.translation();
    const auto M_FGo_G = M_FFo_G.Shift(p_FoGo_G);
    return M_FGo_G;
  };

  // Shift and re-express in G frame the finger's spatial inertias.
  const auto M_LGo_G = CalcFingerSpatialInertiaInGripperFrame(M_LLo_L, X_GL);
  const auto M_RGo_G = CalcFingerSpatialInertiaInGripperFrame(M_RRo_R, X_GR);

  // With everything about the same point Go and expressed in the same frame G,
  // proceed to compose into composite body C:
  // TODO(amcastro-tri): Implement operator+() in SpatialInertia.
  SpatialInertia<double> M_CGo_G = M_GGo_G;
  M_CGo_G += M_LGo_G;
  M_CGo_G += M_RGo_G;

  return M_CGo_G;
}

// Helper factory to make a model of the Kuka arm with a "lumped" (or composite)
// model of the gripper. That is, to control the motion of the arm, we only care
// about a "frozen" gripper, without extra dofs for the gripper.
std::unique_ptr<MultibodyPlant<double>> MakePlantWithCompositeGripper(
    SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>();
  ModelInstanceIndex arm_model =
      AddModelFromSdfFile(FindResourceOrThrow(kSdfPath), plant.get(), scene_graph);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("iiwa_link_0"));

  // Pose of the gripper G in the end effector frame E.
  Isometry3<double> X_EG = Isometry3<double>::Identity();
  X_EG.translation() = Vector3<double>(0, 0, 0.081);
  X_EG.linear() = RollPitchYaw<double>(M_PI_2, 0, M_PI_2).ToRotationMatrix().matrix();

  // Add a "lumped" model of the gripper.
  const auto& gripper = plant->AddRigidBody(
      "gripper", arm_model, MakeCompositeGripperInertia());

  // Weld gripper to end effector.
  const auto& end_effector = plant->GetFrameByName("iiwa_link_7");
  plant->WeldFrames(end_effector, gripper.body_frame());

  // Add gravity to the model.
  plant->AddForceElement<UniformGravityFieldElement>(
      -9.81 * Vector3<double>::UnitZ());

  // Now the model is complete.
  plant->Finalize(scene_graph);

  return plant;
}

int DoMain() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Make and add the kuka robot model.
  MultibodyPlant<double>& kuka_plant =
      *builder.AddSystem(MakePlantWithCompositeGripper(&scene_graph));

  DRAKE_THROW_UNLESS(kuka_plant.num_positions() == 7);
  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!kuka_plant.get_source_id());

  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  auto controller = builder.AddSystem<
      systems::controllers::InverseDynamicsController>(
      kuka_plant,
      iiwa_kp, iiwa_ki, iiwa_kd,
      false /* no feedforward acceleration */);

  // Wire up Kuka plant to controller.
  builder.Connect(kuka_plant.get_continuous_state_output_port(),
                  controller->get_input_port_estimated_state());
  builder.Connect(controller->get_output_port_control(),
                  kuka_plant.get_actuation_input_port());

  // Wire up output from planned trajectory to controller.
  PiecewisePolynomial<double> traj = MakeControlledKukaPlan();
  auto traj_src =
      builder.AddSystem<systems::TrajectorySource<double>>(
          traj, 1 /* outputs q + v */);
  traj_src->set_name("trajectory_source");

  builder.Connect(traj_src->get_output_port(),
                  controller->get_input_port_desired_state());

  builder.Connect(
      kuka_plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(kuka_plant.get_source_id().value()));

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.Initialize();
  simulator.set_target_realtime_rate(1.0);
  simulator.get_mutable_integrator()->set_target_accuracy(1e-3);

  simulator.StepTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
