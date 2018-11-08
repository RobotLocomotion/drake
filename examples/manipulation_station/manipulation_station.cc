#include "drake/examples/manipulation_station/manipulation_station.h"

#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/parsing/multibody_plant_sdf_parser.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/sensors/dev/rgbd_camera.h"

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
using math::RotationMatrix;
using multibody::Joint;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using multibody::SpatialInertia;
using multibody::multibody_plant::MultibodyPlant;
using multibody::parsing::AddModelFromSdfFile;

const int kNumDofIiwa = 7;

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
    const RigidTransform<double> X_GP(
        slider.frame_on_parent().GetFixedPoseInBodyFrame());
    // Pose of the joint's child frame C (attached on the slider's finger body)
    // in the frame of the slider's finger F.
    const RigidTransform<double> X_FC(
        slider.frame_on_child().GetFixedPoseInBodyFrame());
    // When the slider's translational dof is zero, then P coincides with C.
    // Therefore:
    const RigidTransform<double> X_GF = X_GP * X_FC.inverse();
    return X_GF;
  };
  // Pose of left finger L in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GL(CalcFingerPoseInGripperFrame(left_slider));
  // Pose of right finger R in gripper frame G when the slider's dof is zero.
  const RigidTransform<double> X_GR(CalcFingerPoseInGripperFrame(right_slider));
  // Helper to compute the spatial inertia of a finger F in about the gripper's
  // origin Go, expressed in G.
  auto CalcFingerSpatialInertiaInGripperFrame = [](
      const SpatialInertia<double>& M_FFo_F,
      const RigidTransform<double>& X_GF) {
    const auto M_FFo_G = M_FFo_F.ReExpress(X_GF.rotation());
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
ManipulationStation<T>::ManipulationStation(double time_step,
                                            IiwaCollisionModel collision_model)
    : owned_plant_(std::make_unique<MultibodyPlant<T>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()),
      owned_controller_plant_(std::make_unique<MultibodyPlant<T>>()) {
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);

  // Add the table and 80/20 workcell frame.
  const double dx_table_center_to_robot_base = 0.3257;
  const double dz_table_top_robot_base = 0.0127;
  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/manipulation_station/models/amazon_table_simplified.sdf");
  const auto table =
      AddModelFromSdfFile(table_sdf_path, "table", plant_);
  plant_->WeldFrames(
      plant_->world_frame(), plant_->GetFrameByName("amazon_table", table),
      RigidTransform<double>(
          Vector3d(dx_table_center_to_robot_base, 0, -dz_table_top_robot_base))
          .GetAsIsometry3());

  // Add the Kuka IIWA.
  std::string iiwa_sdf_path;
  switch (collision_model) {
    case IiwaCollisionModel::kNoCollision:
      iiwa_sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_no_collision.sdf");
      break;
    case IiwaCollisionModel::kBoxCollision:
      iiwa_sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_with_box_collision.sdf");
      break;
    default:
      DRAKE_ABORT_MSG("Unrecognized collision_model.");
  }

  iiwa_model_ = AddModelFromSdfFile(iiwa_sdf_path, "iiwa", plant_);
  plant_->WeldFrames(plant_->world_frame(),
                     plant_->GetFrameByName("iiwa_link_0", iiwa_model_));

  // Add the Schunk gripper and weld it to the end of the IIWA.
  const std::string wsg_sdf_path = FindResourceOrThrow(
      "drake/manipulation/models/"
      "wsg_50_description/sdf/schunk_wsg_50.sdf");
  wsg_model_ =
      AddModelFromSdfFile(wsg_sdf_path, "gripper", plant_);
  const RigidTransform<double> wsg_pose(RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
                                        Vector3d(0, 0, 0.114));
  plant_->WeldFrames(plant_->GetFrameByName("iiwa_link_7", iiwa_model_),
                     plant_->GetFrameByName("body", wsg_model_),
                     wsg_pose.GetAsIsometry3());

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
                                      wsg_equivalent.body_frame(),
                                      wsg_pose.GetAsIsometry3());

  owned_controller_plant_
      ->template AddForceElement<multibody::UniformGravityFieldElement>(
          -9.81 * Vector3d::UnitZ());
}

template <typename T>
void ManipulationStation<T>::AddCupboard() {
  const double dx_table_center_to_robot_base = 0.3257;
  const double dz_table_top_robot_base = 0.0127;
  const double dx_cupboard_to_table_center = 0.43 + 0.15;
  const double dz_cupboard_to_table_center = 0.02;
  const double cupboard_height = 0.815;

  const std::string sdf_path = FindResourceOrThrow(
      "drake/examples/manipulation_station/models/cupboard.sdf");
  const auto cupboard =
      AddModelFromSdfFile(sdf_path, "cupboard", plant_);
  plant_->WeldFrames(
      plant_->world_frame(), plant_->GetFrameByName("cupboard_body", cupboard),
      RigidTransform<double>(
          RotationMatrix<double>::MakeZRotation(M_PI),
          Vector3d(dx_table_center_to_robot_base + dx_cupboard_to_table_center,
                   0, dz_cupboard_to_table_center + cupboard_height / 2.0 -
                          dz_table_top_robot_base))
          .GetAsIsometry3());
}

template <typename T>
void ManipulationStation<T>::Finalize() {
  // Note: This deferred diagram construction method/workflow exists because we
  //   - cannot finalize plant until all of my objects are added, and
  //   - cannot wire up my diagram until we have finalized the plant.

  plant_->Finalize();

  systems::DiagramBuilder<T> builder;

  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());

  // Export the commanded positions via a PassThrough.
  auto iiwa_position =
      builder.template AddSystem<systems::PassThrough>(kNumDofIiwa);
  builder.ExportInput(iiwa_position->get_input_port(), "iiwa_position");
  builder.ExportOutput(iiwa_position->get_output_port(),
                       "iiwa_position_commanded");

  // Export iiwa "state" outputs.
  {
    auto demux = builder.template AddSystem<systems::Demultiplexer>(
        2 * kNumDofIiwa, kNumDofIiwa);
    builder.Connect(plant_->get_continuous_state_output_port(iiwa_model_),
                    demux->get_input_port(0));
    builder.ExportOutput(demux->get_output_port(0), "iiwa_position_measured");
    builder.ExportOutput(demux->get_output_port(1), "iiwa_velocity_estimated");

    builder.ExportOutput(plant_->get_continuous_state_output_port(iiwa_model_),
                         "iiwa_state_estimated");
  }

  // Add the IIWA controller "stack".
  {
    owned_controller_plant_->Finalize();

    // Add the inverse dynamics controller.
    VectorXd iiwa_kp = VectorXd::Constant(kNumDofIiwa, 100);
    VectorXd iiwa_kd(kNumDofIiwa);
    for (int i = 0; i < kNumDofIiwa; i++) {
      // Critical damping gains.
      iiwa_kd[i] = 2 * std::sqrt(iiwa_kp[i]);
    }
    VectorXd iiwa_ki = VectorXd::Constant(kNumDofIiwa, 1);
    auto iiwa_controller = builder.template AddSystem<
        systems::controllers::InverseDynamicsController>(
        *owned_controller_plant_, iiwa_kp, iiwa_ki, iiwa_kd, false);
    iiwa_controller->set_name("iiwa_controller");
    builder.Connect(plant_->get_continuous_state_output_port(iiwa_model_),
                    iiwa_controller->get_input_port_estimated_state());

    // Add in feedforward torque.
    auto adder = builder.template AddSystem<systems::Adder>(2, kNumDofIiwa);
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
    MatrixXd C(2 * kNumDofIiwa, kNumDofIiwa), D(2 * kNumDofIiwa, kNumDofIiwa);
    // clang-format off
    C << MatrixXd::Zero(kNumDofIiwa, kNumDofIiwa),
         -MatrixXd::Identity(kNumDofIiwa, kNumDofIiwa) / time_step;
    D << MatrixXd::Identity(kNumDofIiwa, kNumDofIiwa),
         MatrixXd::Identity(kNumDofIiwa, kNumDofIiwa) / time_step;
    // clang-format on
    auto desired_state_from_position =
        builder.template AddSystem<systems::LinearSystem>(
            MatrixXd::Zero(kNumDofIiwa, kNumDofIiwa),      // A = 0
            MatrixXd::Identity(kNumDofIiwa, kNumDofIiwa),  // B = I
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

  {
    auto wsg_controller = builder.template AddSystem<
        manipulation::schunk_wsg::SchunkWsgPositionController>();
    wsg_controller->set_name("wsg_controller");

    builder.Connect(wsg_controller->get_generalized_force_output_port(),
                    plant_->get_actuation_input_port(wsg_model_));
    builder.Connect(plant_->get_continuous_state_output_port(wsg_model_),
                    wsg_controller->get_state_input_port());

    builder.ExportInput(wsg_controller->get_desired_position_input_port(),
                        "wsg_position");
    builder.ExportInput(wsg_controller->get_force_limit_input_port(),
                        "wsg_force_limit");

    auto wsg_mbp_state_to_wsg_state = builder.template AddSystem(
        manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem<double>());
    builder.Connect(plant_->get_continuous_state_output_port(wsg_model_),
                    wsg_mbp_state_to_wsg_state->get_input_port());

    builder.ExportOutput(wsg_mbp_state_to_wsg_state->get_output_port(),
                         "wsg_state_measured");

    builder.ExportOutput(wsg_controller->get_grip_force_output_port(),
                         "wsg_force_measured");
  }

  builder.ExportOutput(
      plant_->get_generalized_contact_forces_output_port(iiwa_model_),
      "iiwa_torque_external");

  {  // RGB-D Cameras
    auto render_scene_graph =
        builder.template AddSystem<geometry::dev::SceneGraph>(*scene_graph_);

    builder.Connect(plant_->get_geometry_poses_output_port(),
                    render_scene_graph->get_source_pose_port(
                        plant_->get_source_id().value()));

    geometry::dev::render::DepthCameraProperties camera_properties(
        640, 480, M_PI_4, geometry::dev::render::Fidelity::kLow, 0.1, 2.0);

    // Create the cameras.
    for (int i = 0; i < 3; i++) {
      auto camera =
          builder.template AddSystem<systems::sensors::dev::RgbdCamera>(
              "camera" + std::to_string(i),
              geometry::dev::SceneGraph<double>::world_frame_id(),
              get_camera_pose(i), camera_properties, false);
      builder.Connect(render_scene_graph->get_query_output_port(),
                      camera->query_object_input_port());

      // TODO(russt): Add additional cameras.
      builder.ExportOutput(camera->color_image_output_port(),
                           "camera" + std::to_string(i) + "_rgb_image");
      builder.ExportOutput(camera->depth_image_output_port(),
                           "camera" + std::to_string(i) + "_depth_image");
      builder.ExportOutput(camera->label_image_output_port(),
                           "camera" + std::to_string(i) + "_label_image");
    }
  }

  builder.ExportOutput(scene_graph_->get_pose_bundle_output_port(),
                       "pose_bundle");

  builder.ExportOutput(plant_->get_contact_results_output_port(),
      "contact_results");
  builder.ExportOutput(plant_->get_continuous_state_output_port(),
      "plant_continuous_state");

  builder.BuildInto(this);
}

template <typename T>
VectorX<T> ManipulationStation<T>::GetIiwaPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  // TODO(russt): update upon resolution of #9623.
  VectorX<T> q(kNumDofIiwa);
  for (int i = 0; i < kNumDofIiwa; i++) {
    q(i) = plant_
               ->template GetJointByName<RevoluteJoint>("iiwa_joint_" +
                                                        std::to_string(i + 1))
               .get_angle(plant_context);
  }
  return q;
}

template <typename T>
void ManipulationStation<T>::SetIiwaPosition(
    const Eigen::Ref<const drake::VectorX<T>>& q,
    drake::systems::Context<T>* station_context) const {
  DRAKE_DEMAND(station_context != nullptr);
  DRAKE_DEMAND(q.size() == kNumDofIiwa);
  auto& plant_context =
      this->GetMutableSubsystemContext(*plant_, station_context);
  // TODO(russt): update upon resolution of #9623.
  for (int i = 0; i < kNumDofIiwa; i++) {
    plant_
        ->template GetJointByName<RevoluteJoint>("iiwa_joint_" +
                                                 std::to_string(i + 1))
        .set_angle(&plant_context, q(i));
  }

  // Set the position history in the state interpolator to match.
  this->GetMutableSubsystemContext(
          this->GetSubsystemByName("desired_state_from_position"),
          station_context)
      .get_mutable_discrete_state_vector()
      .SetFromVector(q);
}

template <typename T>
VectorX<T> ManipulationStation<T>::GetIiwaVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  VectorX<T> v(kNumDofIiwa);
  // TODO(russt): update upon resolution of #9623.
  for (int i = 0; i < kNumDofIiwa; i++) {
    v(i) = plant_
               ->template GetJointByName<RevoluteJoint>("iiwa_joint_" +
                                                        std::to_string(i + 1))
               .get_angular_rate(plant_context);
  }
  return v;
}

template <typename T>
void ManipulationStation<T>::SetIiwaVelocity(
    const Eigen::Ref<const drake::VectorX<T>>& v,
    drake::systems::Context<T>* station_context) const {
  DRAKE_DEMAND(station_context != nullptr);
  DRAKE_DEMAND(v.size() == kNumDofIiwa);
  auto& plant_context =
      this->GetMutableSubsystemContext(*plant_, station_context);
  // TODO(russt): update upon resolution of #9623.
  for (int i = 0; i < kNumDofIiwa; i++) {
    plant_
        ->template GetJointByName<RevoluteJoint>("iiwa_joint_" +
                                                 std::to_string(i + 1))
        .set_angular_rate(&plant_context, v(i));
  }
}

template <typename T>
T ManipulationStation<T>::GetWsgPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  // TODO(russt): update upon resolution of #9623.
  return plant_
             ->template GetJointByName<PrismaticJoint>(
                 "right_finger_sliding_joint", wsg_model_)
             .get_translation(plant_context) -
         plant_
             ->template GetJointByName<PrismaticJoint>(
                 "left_finger_sliding_joint", wsg_model_)
             .get_translation(plant_context);
}

template <typename T>
T ManipulationStation<T>::GetWsgVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  // TODO(russt): update upon resolution of #9623.
  return plant_
             ->template GetJointByName<PrismaticJoint>(
                 "right_finger_sliding_joint", wsg_model_)
             .get_translation_rate(plant_context) -
         plant_
             ->template GetJointByName<PrismaticJoint>(
                 "left_finger_sliding_joint", wsg_model_)
             .get_translation_rate(plant_context);
}

template <typename T>
void ManipulationStation<T>::SetWsgPosition(
    const T& q, drake::systems::Context<T>* station_context) const {
  auto& plant_context =
      this->GetMutableSubsystemContext(*plant_, station_context);

  // TODO(russt): update upon resolution of #9623.
  plant_
      ->template GetJointByName<PrismaticJoint>("right_finger_sliding_joint",
                                                wsg_model_)
      .set_translation(&plant_context, q / 2);
  plant_
      ->template GetJointByName<PrismaticJoint>("left_finger_sliding_joint",
                                                wsg_model_)
      .set_translation(&plant_context, -q / 2);

  // Set the position history in the state interpolator to match.
  const auto& wsg_controller = dynamic_cast<
      const manipulation::schunk_wsg::SchunkWsgPositionController&>(
      this->GetSubsystemByName("wsg_controller"));
  wsg_controller.set_desired_position_history(q,
      &this->GetMutableSubsystemContext(wsg_controller, station_context));
}

template <typename T>
void ManipulationStation<T>::SetWsgVelocity(
    const T& v, drake::systems::Context<T>* station_context) const {
  auto& plant_context =
      this->GetMutableSubsystemContext(*plant_, station_context);

  // TODO(russt): update upon resolution of #9623.
  plant_
      ->template GetJointByName<PrismaticJoint>("right_finger_sliding_joint",
                                                wsg_model_)
      .set_translation_rate(&plant_context, v / 2);
  plant_
      ->template GetJointByName<PrismaticJoint>("left_finger_sliding_joint",
                                                wsg_model_)
      .set_translation_rate(&plant_context, -v / 2);
}

template <typename T>
Isometry3d ManipulationStation<T>::get_camera_pose(int camera_number) {
  DRAKE_DEMAND(camera_number >= 0 && camera_number <= 2);

  Vector3d p_WC;
  Vector3d rpy_WC;

  switch (camera_number) {
    case 0:
      p_WC << -0.233066, -0.451461, 0.466761;
      rpy_WC << 1.69101, 0.176488, 0.432721;
      break;
    case 1:
      p_WC << -0.197236, 0.468471, 0.436499;
      rpy_WC << -1.68974, 0.20245, -0.706783;
      break;
    case 2:
      p_WC << 0.786905, -0.0284378, 1.04287;
      rpy_WC << 0.0438918, 1.03776, -3.13612;
      break;
  }
  return RigidTransform<double>(RollPitchYaw<double>(rpy_WC), p_WC)
      .GetAsIsometry3();
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

// TODO(russt): Support at least NONSYMBOLIC_SCALARS.  See #9573.
//   (and don't forget to include default_scalars.h)
template class ::drake::examples::manipulation_station::ManipulationStation<
    double>;
