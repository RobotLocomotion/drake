#include "drake/examples/manipulation_station/station_simulation.h"

#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace examples {
namespace manipulation_station {

using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::SceneGraph;
using math::RigidTransform;
using math::RollPitchYaw;
using multibody::Joint;
using multibody::SpatialInertia;
using multibody::multibody_plant::MultibodyPlant;
using multibody::parsing::AddModelFromSdfFile;

// TODO(amcastro-tri): Refactor this into schunk_wsg directory, and cover it
// with a unit test.  Potentially tighten the tolerance in
// station_simulation_test.
SpatialInertia<double> MakeCompositeGripperInertia(
    const std::string& wsg_sdf_path) {
  MultibodyPlant<double> plant;
  AddModelFromSdfFile(wsg_sdf_path, &plant);
  plant.Finalize();
  const auto& gripper_body = plant.tree().GetRigidBodyByName("body");
  const auto& left_finger = plant.tree().GetRigidBodyByName("left_finger");
  const auto& right_finger = plant.tree().GetRigidBodyByName("right_finger");
  const auto& left_slider = plant.GetJointByName("left_finger_sliding_joint");
  const auto& right_slider = plant.GetJointByName("right_finger_sliding_joint");
  const SpatialInertia<double>& M_GGo_G =
      gripper_body.default_spatial_inertia();
  const SpatialInertia<double>& M_LLo_L = left_finger.default_spatial_inertia();
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

template <typename T>
StationSimulation<T>::StationSimulation(double time_step)
    : owned_plant_(std::make_unique<MultibodyPlant<T>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()),
      owned_controller_plant_(std::make_unique<MultibodyPlant<T>>()) {
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();

  // Add a table for the robot.
  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");
  const auto robot_table =
      AddModelFromSdfFile(table_sdf_path, "robot_table", plant_, scene_graph_);
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("link", robot_table));

  // Add the Kuka IIWA.
  const std::string iiwa_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/"
      "sdf/iiwa14_no_collision.sdf");
  iiwa_model_ =
      AddModelFromSdfFile(iiwa_sdf_path, "iiwa", plant_, scene_graph_);
  const double table_top_z_in_world = 0.736 + 0.057 / 2;
  plant_->WeldFrames(
      plant_->world_frame(), plant_->GetFrameByName("iiwa_link_0", iiwa_model_),
      RigidTransform<double>(Vector3d(0, 0, table_top_z_in_world))
          .GetAsIsometry3());

  // Add the Schunk gripper and weld it to the end of the IIWA.
  const std::string wsg_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/"
      "wsg_50_description/sdf/schunk_wsg_50.sdf");
  wsg_model_ =
      AddModelFromSdfFile(wsg_sdf_path, "gripper", plant_, scene_graph_);
  const Isometry3d wsg_pose =
      RigidTransform<double>(
          RollPitchYaw<double>(M_PI_2, 0, M_PI_2).ToRotationMatrix(),
          Vector3d(0, 0, 0.081))
          .GetAsIsometry3();
  plant_->WeldFrames(plant_->GetFrameByName("iiwa_link_7", iiwa_model_),
                     plant_->GetFrameByName("body", wsg_model_), wsg_pose);

  // Add a second table that is the main robot workspace.
  const auto table =
      AddModelFromSdfFile(table_sdf_path, "table", plant_, scene_graph_);
  plant_->WeldFrames(
      plant_->world_frame(), plant_->GetFrameByName("link", table),
      RigidTransform<double>(Vector3d(0.8, 0, 0.0)).GetAsIsometry3());

  plant_->template AddForceElement<multibody::UniformGravityFieldElement>(
      -9.81 * Vector3d::UnitZ());

  // Build the controller's version of the plant, which only contains the
  // IIWA and the equivalent inertia of the gripper.
  const auto controller_iiwa_model =
      AddModelFromSdfFile(iiwa_sdf_path, "iiwa", owned_controller_plant_.get());
  owned_controller_plant_->WeldFrames(owned_controller_plant_->world_frame(),
                                      owned_controller_plant_->GetFrameByName(
                                          "iiwa_link_0", controller_iiwa_model),
                                      Isometry3d::Identity());
  // Add a single body to represent the IIWA pendant's calibration of the
  // gripper.  The body of the WSG accounts for >90% of the total mass
  // (according to the sdf)... and we don't believe our inertia calibration
  // on the hardware to be so precise, so we simply ignore the inertia
  // contribution from the fingers here.
  const multibody::RigidBody<T>& wsg_equivalent =
      owned_controller_plant_->AddRigidBody(
          "wsg_equivalent", controller_iiwa_model,
          MakeCompositeGripperInertia(wsg_sdf_path));
  owned_controller_plant_->WeldFrames(owned_controller_plant_->GetFrameByName(
                                          "iiwa_link_7", controller_iiwa_model),
                                      wsg_equivalent.body_frame(), wsg_pose);

  owned_controller_plant_
      ->template AddForceElement<multibody::UniformGravityFieldElement>(
          -9.81 * Vector3d::UnitZ());
}

template <typename T>
void StationSimulation<T>::Finalize() {
  // Note: This deferred diagram construction method/workflow exists because we
  //   - cannot finalize plant until all of my objects are added, and
  //   - cannot wire up my diagram until we have finalized the plant.

  const int kDoF = 7;
  plant_->Finalize(scene_graph_);

  systems::DiagramBuilder<T> builder;

  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());

  // Export the commanded positions via a PassThrough.
  auto iiwa_position = builder.template AddSystem<systems::PassThrough>(kDoF);
  builder.ExportInput(iiwa_position->get_input_port(), "iiwa_position");
  builder.ExportOutput(iiwa_position->get_output_port(),
                       "iiwa_position_commanded");

  // Export iiwa "state" outputs.
  {
    auto demux =
        builder.template AddSystem<systems::Demultiplexer>(2 * kDoF, kDoF);
    builder.Connect(plant_->get_continuous_state_output_port(iiwa_model_),
                    demux->get_input_port(0));
    builder.ExportOutput(demux->get_output_port(0), "iiwa_position_measured");
    builder.ExportOutput(demux->get_output_port(1), "iiwa_velocity_estimated");

    builder.ExportOutput(plant_->get_continuous_state_output_port
        (iiwa_model_), "iiwa_state_estimated");
  }

  // Add the IIWA controller "stack".
  {
    owned_controller_plant_->Finalize();

    // Add the inverse dynamics controller.
    VectorX<double> iiwa_kp = VectorXd::Constant(kDoF, 100);
    VectorX<double> iiwa_kd(kDoF);
    for (int i = 0; i < kDoF; i++) {
      // Critical damping gains.
      iiwa_kd[i] = 2 * std::sqrt(iiwa_kp[i]);
    }
    VectorX<double> iiwa_ki = VectorXd::Constant(kDoF, 1);
    auto iiwa_controller = builder.template AddSystem<
        systems::controllers::InverseDynamicsController>(
        *owned_controller_plant_, iiwa_kp, iiwa_ki, iiwa_kd, false);
    iiwa_controller->set_name("iiwa_controller");
    builder.Connect(plant_->get_continuous_state_output_port(iiwa_model_),
                    iiwa_controller->get_input_port_estimated_state());

    // Add in feedforward torque.
    auto adder = builder.template AddSystem<systems::Adder>(2, kDoF);
    builder.Connect(iiwa_controller->get_output_port_control(),
                    adder->get_input_port(0));
    builder.ExportInput(adder->get_input_port(1), "iiwa_feedforward_torque");
    builder.Connect(adder->get_output_port(),
                    plant_->get_actuation_input_port(iiwa_model_));

    // Approximate desired state command from a discrete derivative of the
    // position command input port.  This is implemented as a LinearSystem
    // with state variables to store the last position command.
    //    x[n+1] = u[n]
    //    y[n] = [u[n]; (u[n] - x[n])/h]
    // where u[n] is the positions, y[n] output the positions and
    // velocities, and h is the timestep.
    const double time_step = plant_->time_step();
    MatrixXd C(2 * kDoF, kDoF), D(2 * kDoF, kDoF);
    // clang-format off
    C << MatrixXd::Zero(kDoF, kDoF),
         -MatrixXd::Identity(kDoF, kDoF) / time_step;
    D << MatrixXd::Identity(kDoF, kDoF),
         MatrixXd::Identity(kDoF, kDoF) / time_step;
    // clang-format on
    auto desired_state_from_position =
        builder.template AddSystem<systems::LinearSystem>(
            MatrixXd::Zero(kDoF, kDoF),      // A = 0
            MatrixXd::Identity(kDoF, kDoF),  // B = I
            C, D, time_step);
    desired_state_from_position->set_name("desired_state_from_position");
    builder.Connect(desired_state_from_position->get_output_port(),
                    iiwa_controller->get_input_port_desired_state());
    builder.Connect(iiwa_position->get_output_port(),
                    desired_state_from_position->get_input_port());

    // Export commanded torques:
    builder.ExportOutput(adder->get_output_port(), "iiwa_torque_commanded");
    builder.ExportOutput(adder->get_output_port(), "iiwa_torque_measured");
  }

  // TODO(russt): Add the WSG controller "stack".
  {
    // For now, just send zero force inputs.
    const auto wsg_command =
        builder.template AddSystem<systems::ConstantVectorSource>(
            Eigen::Vector2d::Zero());
    builder.Connect(wsg_command->get_output_port(),
                    plant_->get_actuation_input_port(wsg_model_));
  }

  builder.ExportOutput(
      plant_->get_generalized_contact_forces_output_port(iiwa_model_),
      "iiwa_torque_external");
  builder.ExportOutput(scene_graph_->get_pose_bundle_output_port(),
                       "pose_bundle");

  builder.BuildInto(this);
}

template <typename T>
VectorX<T> StationSimulation<T>::GetIiwaPosition(
    const systems::Context<T>& station_context) const {
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  // This assumes that the IIWA state comes first.  A unit test in
  // station_simulation_test.cc validates this assumption.
  // TODO(russt): update upon resolution of #9623.
  return plant_->tree()
      .get_multibody_state_vector(plant_context)
      .template segment<7>(0);
}

template <typename T>
void StationSimulation<T>::SetIiwaPosition(
    const Eigen::Ref<const drake::VectorX<T>>& q,
    drake::systems::Context<T>* station_context) const {
  DRAKE_DEMAND(station_context != nullptr);
  DRAKE_DEMAND(q.size() == 7);
  auto& plant_context =
      this->GetMutableSubsystemContext(*plant_, station_context);
  // This assumes that the IIWA state comes first.  A unit test in
  // station_simulation_test.cc validates this assumption.
  // TODO(russt): update upon resolution of #9623.
  plant_->tree()
      .get_mutable_multibody_state_vector(&plant_context)
      .template segment<7>(0) = q;

  // Set the position history in the state interpolator to match.
  this->GetMutableSubsystemContext(
          this->GetSubsystemByName("desired_state_from_position"),
          station_context)
      .get_mutable_discrete_state_vector()
      .SetFromVector(q);
}

template <typename T>
VectorX<T> StationSimulation<T>::GetIiwaVelocity(
    const systems::Context<T>& station_context) const {
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  // This assumes that the IIWA state comes first, and that velocities follow
  // positions.  A unit test in station_simulation_test.cc validates this
  // assumption.
  // TODO(russt): update upon resolution of #9623.
  return plant_->tree()
      .get_multibody_state_vector(plant_context)
      .template segment<7>(plant_->num_positions());
}

template <typename T>
void StationSimulation<T>::SetIiwaVelocity(
    const Eigen::Ref<const drake::VectorX<T>>& v,
    drake::systems::Context<T>* station_context) const {
  DRAKE_DEMAND(station_context != nullptr);
  DRAKE_DEMAND(v.size() == 7);
  auto& plant_context =
      this->GetMutableSubsystemContext(*plant_, station_context);
  // This assumes that the IIWA state comes first.  A unit test in
  // station_simulation_test.cc validates this assumption.
  // TODO(russt): update upon resolution of #9623.
  plant_->tree()
      .get_mutable_multibody_state_vector(&plant_context)
      .template segment<7>(plant_->num_positions()) = v;
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

// TODO(russt): Support at least NONSYMBOLIC_SCALARS.  See #9573.
//   (and don't forget to include default_scalars.h)
template class ::drake::examples::manipulation_station::StationSimulation<
    double>;
