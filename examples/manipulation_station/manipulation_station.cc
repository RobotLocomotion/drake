#include "drake/examples/manipulation_station/manipulation_station.h"

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_constants.h"
#include "drake/manipulation/schunk_wsg/schunk_wsg_position_controller.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/discrete_derivative.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/sensors/rgbd_sensor.h"

namespace drake {
namespace examples {
namespace manipulation_station {

using Eigen::Vector3d;
using Eigen::VectorXd;
using geometry::SceneGraph;
using geometry::render::MakeRenderEngineVtk;
using geometry::render::RenderEngineVtkParams;
using math::RigidTransform;
using math::RigidTransformd;
using math::RollPitchYaw;
using math::RotationMatrix;
using multibody::Joint;
using multibody::MultibodyPlant;
using multibody::PrismaticJoint;
using multibody::RevoluteJoint;
using multibody::SpatialInertia;

namespace internal {

// TODO(amcastro-tri): Refactor this into schunk_wsg directory, and cover it
// with a unit test.  Potentially tighten the tolerance in
// station_simulation_test.
// @param gripper_body_frame_name Name of a frame that's attached to the
// gripper's main body.
SpatialInertia<double> MakeCompositeGripperInertia(
    const std::string& wsg_sdf_path,
    const std::string& gripper_body_frame_name) {
  // Set timestep to 1.0 since it is arbitrary, to quiet joint limit warnings.
  MultibodyPlant<double> plant(1.0);
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(wsg_sdf_path);
  plant.Finalize();
  const auto& frame = plant.GetFrameByName(gripper_body_frame_name);
  const auto& gripper_body = plant.GetRigidBodyByName(frame.body().name());
  const auto& left_finger = plant.GetRigidBodyByName("left_finger");
  const auto& right_finger = plant.GetRigidBodyByName("right_finger");
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
  auto CalcFingerSpatialInertiaInGripperFrame =
      [](const SpatialInertia<double>& M_FFo_F,
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

// TODO(russt): Get these from SDF instead of having them hard-coded (#10022).
void get_camera_poses(std::map<std::string, RigidTransform<double>>* pose_map) {
  pose_map->emplace("0", RigidTransform<double>(
                             RollPitchYaw<double>(2.549607, 1.357609, 2.971679),
                             Vector3d(-0.228895, -0.452176, 0.486308)));

  pose_map->emplace("1",
                    RigidTransform<double>(
                        RollPitchYaw<double>(2.617427, -1.336404, -0.170522),
                        Vector3d(-0.201813, 0.469259, 0.417045)));

  pose_map->emplace("2",
                    RigidTransform<double>(
                        RollPitchYaw<double>(-2.608978, 0.022298, 1.538460),
                        Vector3d(0.786258, -0.048422, 1.043315)));
}

// Load a SDF model and weld it to the MultibodyPlant.
// @param model_path Full path to the sdf model file. i.e. with
// FindResourceOrThrow
// @param model_name Name of the added model instance.
// @param parent Frame P from the MultibodyPlant to which the new model is
// welded to.
// @param child_frame_name Defines frame C (the child frame), assumed to be
// present in the model being added.
// @param X_PC Transformation of frame C relative to frame P.
template <typename T>
multibody::ModelInstanceIndex AddAndWeldModelFrom(
    const std::string& model_path, const std::string& model_name,
    const multibody::Frame<T>& parent, const std::string& child_frame_name,
    const RigidTransform<double>& X_PC, MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(!plant->HasModelInstanceNamed(model_name));

  multibody::Parser parser(plant);
  const multibody::ModelInstanceIndex new_model =
      parser.AddModelFromFile(model_path, model_name);
  const auto& child_frame = plant->GetFrameByName(child_frame_name, new_model);
  plant->WeldFrames(parent, child_frame, X_PC);
  return new_model;
}

std::pair<geometry::render::ColorRenderCamera,
          geometry::render::DepthRenderCamera>
MakeD415CameraModel(const std::string& renderer_name) {
  // Typical D415 intrinsics for 848 x 480 resolution, note that rgb and
  // depth are slightly different (in both intrinsics and relative to the
  // camera body frame).
  // RGB:
  // - w: 848, h: 480, fx: 616.285, fy: 615.778, ppx: 405.418, ppy: 232.864
  // DEPTH:
  // - w: 848, h: 480, fx: 645.138, fy: 645.138, ppx: 420.789, ppy: 239.13
  const int kHeight = 480;
  const int kWidth = 848;

  // To pose the two sensors relative to the camera body, we'll assume X_BC = I,
  // and select a representative value for X_CD drawn from calibration to define
  // X_BD.
  geometry::render::ColorRenderCamera color_camera{
      {renderer_name,
       {kWidth, kHeight, 616.285, 615.778, 405.418, 232.864} /* intrinsics */,
       {0.01, 3.0} /* clipping_range */,
       {} /* X_BC */},
      false};
  const RigidTransformd X_BD(
      RotationMatrix<double>(RollPitchYaw<double>(
          -0.19 * M_PI / 180, -0.016 * M_PI / 180, -0.03 * M_PI / 180)),
      Vector3d(0.015, -0.00019, -0.0001));
  geometry::render::DepthRenderCamera depth_camera{
      {renderer_name,
       {kWidth, kHeight, 645.138, 645.138, 420.789, 239.13} /* intrinsics */,
       {0.01, 3.0} /* clipping_range */,
       X_BD},
      {0.1, 2.0} /* depth_range */};
  return {color_camera, depth_camera};
}

}  // namespace internal

template <typename T>
ManipulationStation<T>::ManipulationStation(double time_step)
    : owned_plant_(std::make_unique<MultibodyPlant<T>>(time_step)),
      owned_scene_graph_(std::make_unique<SceneGraph<T>>()),
      // Given the controller does not compute accelerations, it is irrelevant
      // whether the plant is continuous or discrete. We make it
      // discrete to avoid warnings about joint limits.
      owned_controller_plant_(std::make_unique<MultibodyPlant<T>>(1.0)) {
  // This class holds the unique_ptrs explicitly for plant and scene_graph
  // until Finalize() is called (when they are moved into the Diagram). Grab
  // the raw pointers, which should stay valid for the lifetime of the Diagram.
  plant_ = owned_plant_.get();
  scene_graph_ = owned_scene_graph_.get();
  plant_->RegisterAsSourceForSceneGraph(scene_graph_);
  scene_graph_->set_name("scene_graph");
  plant_->set_name("plant");

  this->set_name("manipulation_station");
}

template <typename T>
void ManipulationStation<T>::AddManipulandFromFile(
    const std::string& model_file, const RigidTransform<double>& X_WObject) {
  multibody::Parser parser(plant_);
  const auto model_index =
      parser.AddModelFromFile(FindResourceOrThrow(model_file));
  const auto indices = plant_->GetBodyIndices(model_index);
  // Only support single-body objects for now.
  // Note: this could be generalized fairly easily... would just want to
  // set default/random positions for the non-floating-base elements below.
  DRAKE_DEMAND(indices.size() == 1);
  object_ids_.push_back(indices[0]);

  object_poses_.push_back(X_WObject);
}

template <typename T>
void ManipulationStation<T>::SetupClutterClearingStation(
    const std::optional<const math::RigidTransform<double>>& X_WCameraBody,
    IiwaCollisionModel collision_model, SchunkCollisionModel schunk_model) {
  DRAKE_DEMAND(setup_ == Setup::kNone);
  setup_ = Setup::kClutterClearing;

  // Add the bins.
  {
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/bin.sdf");

    RigidTransform<double> X_WC(RotationMatrix<double>::MakeZRotation(M_PI_2),
                                Vector3d(-0.145, -0.63, 0.075));
    internal::AddAndWeldModelFrom(sdf_path, "bin1", plant_->world_frame(),
                                  "bin_base", X_WC, plant_);

    X_WC = RigidTransform<double>(RotationMatrix<double>::MakeZRotation(M_PI),
                                  Vector3d(0.5, -0.1, 0.075));
    internal::AddAndWeldModelFrom(sdf_path, "bin2", plant_->world_frame(),
                                  "bin_base", X_WC, plant_);
  }

  // Add the camera.
  {
    const auto& [color_camera, depth_camera] =
        internal::MakeD415CameraModel(default_renderer_name_);

    RegisterRgbdSensor("0", plant_->world_frame(),
                       X_WCameraBody.value_or(math::RigidTransform<double>(
                           math::RollPitchYaw<double>(-0.3, 0.8, 1.5),
                           Eigen::Vector3d(0, -1.5, 1.5))),
                       color_camera, depth_camera);
  }

  AddDefaultIiwa(collision_model);
  AddDefaultWsg(schunk_model);
}

template <typename T>
void ManipulationStation<T>::SetupManipulationClassStation(
  IiwaCollisionModel collision_model,
  SchunkCollisionModel schunk_model) {
  DRAKE_DEMAND(setup_ == Setup::kNone);
  setup_ = Setup::kManipulationClass;

  // Add the table and 80/20 workcell frame.
  {
    const double dx_table_center_to_robot_base = 0.3257;
    const double dz_table_top_robot_base = 0.0127;
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/"
        "amazon_table_simplified.sdf");

    RigidTransform<double> X_WT(
        Vector3d(dx_table_center_to_robot_base, 0, -dz_table_top_robot_base));
    internal::AddAndWeldModelFrom(sdf_path, "table", plant_->world_frame(),
                                  "amazon_table", X_WT, plant_);
  }

  // Add the cupboard.
  {
    const double dx_table_center_to_robot_base = 0.3257;
    const double dz_table_top_robot_base = 0.0127;
    const double dx_cupboard_to_table_center = 0.43 + 0.15;
    const double dz_cupboard_to_table_center = 0.02;
    const double cupboard_height = 0.815;

    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/manipulation_station/models/cupboard.sdf");

    RigidTransform<double> X_WC(
        RotationMatrix<double>::MakeZRotation(M_PI),
        Vector3d(dx_table_center_to_robot_base + dx_cupboard_to_table_center, 0,
                 dz_cupboard_to_table_center + cupboard_height / 2.0 -
                     dz_table_top_robot_base));
    internal::AddAndWeldModelFrom(sdf_path, "cupboard", plant_->world_frame(),
                                  "cupboard_body", X_WC, plant_);
  }

  // Add the default iiwa/wsg models.
  AddDefaultIiwa(collision_model);
  AddDefaultWsg(schunk_model);

  // Add default cameras.
  {
    std::map<std::string, RigidTransform<double>> camera_poses;
    internal::get_camera_poses(&camera_poses);
    const auto& [color_camera, depth_camera] =
        internal::MakeD415CameraModel(default_renderer_name_);
    for (const auto& camera_pair : camera_poses) {
      RegisterRgbdSensor(camera_pair.first, plant_->world_frame(),
                         camera_pair.second, color_camera, depth_camera);
    }
  }
}

template <typename T>
void ManipulationStation<T>::SetupPlanarIiwaStation(
  SchunkCollisionModel schunk_model) {
  DRAKE_DEMAND(setup_ == Setup::kNone);
  setup_ = Setup::kPlanarIiwa;

  // Add the tables.
  {
    const std::string sdf_path = FindResourceOrThrow(
        "drake/examples/kuka_iiwa_arm/models/table/"
        "extra_heavy_duty_table_surface_only_collision.sdf");

    const double table_height = 0.7645;
    internal::AddAndWeldModelFrom(
        sdf_path, "robot_table", plant_->world_frame(), "link",
        RigidTransform<double>(Vector3d(0, 0, -table_height)), plant_);
    internal::AddAndWeldModelFrom(
        sdf_path, "work_table", plant_->world_frame(), "link",
        RigidTransform<double>(Vector3d(0.75, 0, -table_height)), plant_);
  }

  // Add planar iiwa model.
  {
    std::string sdf_path = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "planar_iiwa14_spheres_dense_elbow_collision.urdf");
    const auto X_WI = RigidTransform<double>::Identity();
    auto iiwa_instance = internal::AddAndWeldModelFrom(
        sdf_path, "iiwa", plant_->world_frame(), "iiwa_link_0", X_WI, plant_);
    RegisterIiwaControllerModel(
        sdf_path, iiwa_instance, plant_->world_frame(),
        plant_->GetFrameByName("iiwa_link_0", iiwa_instance), X_WI);
  }

  // Add the default wsg model.
  AddDefaultWsg(schunk_model);
}

template <typename T>
int ManipulationStation<T>::num_iiwa_joints() const {
  DRAKE_DEMAND(iiwa_model_.model_instance.is_valid());
  return plant_->num_positions(iiwa_model_.model_instance);
}

template <typename T>
void ManipulationStation<T>::SetDefaultState(
    const systems::Context<T>& station_context,
    systems::State<T>* state) const {
  // Call the base class method, to initialize all systems in this diagram.
  systems::Diagram<T>::SetDefaultState(station_context, state);

  T q0_gripper{0.1};

  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  DRAKE_DEMAND(object_ids_.size() == object_poses_.size());

  for (uint64_t i = 0; i < object_ids_.size(); i++) {
    plant_->SetFreeBodyPose(plant_context, &plant_state,
                            plant_->get_body(object_ids_[i]), object_poses_[i]);
  }

  // Use SetIiwaPosition to make sure the controller state is initialized to
  // the IIWA state.
  SetIiwaPosition(station_context, state, GetIiwaPosition(station_context));
  SetIiwaVelocity(station_context, state, VectorX<T>::Zero(num_iiwa_joints()));
  SetWsgPosition(station_context, state, q0_gripper);
  SetWsgVelocity(station_context, state, 0);
}

template <typename T>
void ManipulationStation<T>::SetRandomState(
    const systems::Context<T>& station_context, systems::State<T>* state,
    RandomGenerator* generator) const {
  // Call the base class method, to initialize all systems in this diagram.
  systems::Diagram<T>::SetRandomState(station_context, state, generator);

  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  // Separate the objects by lifting them up in z (in a random order).
  // TODO(russt): Replace this with an explicit projection into a statically
  // stable configuration.
  std::vector<multibody::BodyIndex> shuffled_object_ids(object_ids_);
  std::shuffle(shuffled_object_ids.begin(), shuffled_object_ids.end(),
               *generator);
  double z_offset = 0.1;
  for (const auto& body_index : shuffled_object_ids) {
    math::RigidTransform<T> pose =
        plant_->GetFreeBodyPose(plant_context, plant_->get_body(body_index));
    pose.set_translation(pose.translation() + Vector3d{0, 0, z_offset});
    z_offset += 0.1;
    plant_->SetFreeBodyPose(plant_context, &plant_state,
                            plant_->get_body(body_index), pose);
  }

  // Use SetIiwaPosition to make sure the controller state is initialized to
  // the IIWA state.
  SetIiwaPosition(station_context, state, GetIiwaPosition(station_context));
  SetIiwaVelocity(station_context, state, VectorX<T>::Zero(num_iiwa_joints()));
  SetWsgPosition(station_context, state, GetWsgPosition(station_context));
  SetWsgVelocity(station_context, state, 0);
}

template <typename T>
void ManipulationStation<T>::MakeIiwaControllerModel() {
  // Build the controller's version of the plant, which only contains the
  // IIWA and the equivalent inertia of the gripper.
  multibody::Parser parser(owned_controller_plant_.get());
  const auto controller_iiwa_model =
      parser.AddModelFromFile(iiwa_model_.model_path, "iiwa");

  owned_controller_plant_->WeldFrames(
      owned_controller_plant_->world_frame(),
      owned_controller_plant_->GetFrameByName(iiwa_model_.child_frame->name(),
                                              controller_iiwa_model),
      iiwa_model_.X_PC);
  // Add a single body to represent the IIWA pendant's calibration of the
  // gripper.  The body of the WSG accounts for >90% of the total mass
  // (according to the sdf)... and we don't believe our inertia calibration
  // on the hardware to be so precise, so we simply ignore the inertia
  // contribution from the fingers here.
  const multibody::RigidBody<T>& wsg_equivalent =
      owned_controller_plant_->AddRigidBody(
          "wsg_equivalent", controller_iiwa_model,
          internal::MakeCompositeGripperInertia(
              wsg_model_.model_path, wsg_model_.child_frame->name()));

  // TODO(siyuan.feng@tri.global): when we handle multiple IIWA and WSG, this
  // part need to deal with the parent's (iiwa's) model instance id.
  owned_controller_plant_->WeldFrames(
      owned_controller_plant_->GetFrameByName(wsg_model_.parent_frame->name(),
                                              controller_iiwa_model),
      wsg_equivalent.body_frame(), wsg_model_.X_PC);
  owned_controller_plant_->set_name("controller_plant");
}

template <typename T>
void ManipulationStation<T>::Finalize() {
  Finalize({});
}

template <typename T>
void ManipulationStation<T>::Finalize(
    std::map<std::string, std::unique_ptr<geometry::render::RenderEngine>>
        render_engines) {
  DRAKE_THROW_UNLESS(iiwa_model_.model_instance.is_valid());
  DRAKE_THROW_UNLESS(wsg_model_.model_instance.is_valid());

  MakeIiwaControllerModel();

  // Note: This deferred diagram construction method/workflow exists because we
  //   - cannot finalize plant until all of my objects are added, and
  //   - cannot wire up my diagram until we have finalized the plant.
  plant_->Finalize();

  // Set plant properties that must occur after finalizing the plant.
  VectorX<T> q0_iiwa(num_iiwa_joints());

  switch (setup_) {
    case Setup::kNone:
    case Setup::kManipulationClass: {
      // Set the initial positions of the IIWA to a comfortable configuration
      // inside the workspace of the station.
      q0_iiwa << 0, 0.6, 0, -1.75, 0, 1.0, 0;

      std::uniform_real_distribution<symbolic::Expression> x(0.4, 0.65),
          y(-0.35, 0.35), z(0, 0.05);
      const Vector3<symbolic::Expression> xyz{x(), y(), z()};
      for (const auto& body_index : object_ids_) {
        const multibody::Body<T>& body = plant_->get_body(body_index);
        plant_->SetFreeBodyRandomPositionDistribution(body, xyz);
        plant_->SetFreeBodyRandomRotationDistributionToUniform(body);
      }
      break;
    }
    case Setup::kClutterClearing: {
      // Set the initial positions of the IIWA to a configuration right above
      // the picking bin.
      q0_iiwa << -1.57, 0.1, 0, -1.2, 0, 1.6, 0;

      std::uniform_real_distribution<symbolic::Expression> x(-.35, 0.05),
          y(-0.8, -.55), z(0.3, 0.35);
      const Vector3<symbolic::Expression> xyz{x(), y(), z()};
      for (const auto& body_index : object_ids_) {
        const multibody::Body<T>& body = plant_->get_body(body_index);
        plant_->SetFreeBodyRandomPositionDistribution(body, xyz);
        plant_->SetFreeBodyRandomRotationDistributionToUniform(body);
      }
      break;
    }
    case Setup::kPlanarIiwa: {
      // Set initial positions of the IIWA, but now with only joints 2, 4,
      // and 6.
      q0_iiwa << 0.1, -1.2, 1.6;

      std::uniform_real_distribution<symbolic::Expression> x(0.4, 0.8),
          y(0, 0), z(0, 0.05);
      const Vector3<symbolic::Expression> xyz{x(), y(), z()};
      for (const auto& body_index : object_ids_) {
        const multibody::Body<T>& body = plant_->get_body(body_index);
        plant_->SetFreeBodyRandomPositionDistribution(body, xyz);
      }
      break;
    }
  }

  // Set the iiwa default configuration.
  const auto iiwa_joint_indices =
      plant_->GetJointIndices(iiwa_model_.model_instance);
  int q0_index = 0;
  for (const auto& joint_index : iiwa_joint_indices) {
    multibody::RevoluteJoint<T>* joint =
        dynamic_cast<multibody::RevoluteJoint<T>*>(
            &plant_->get_mutable_joint(joint_index));
    // Note: iiwa_joint_indices includes the WeldJoint at the base.  Only set
    // the RevoluteJoints.
    if (joint) {
      joint->set_default_angle(q0_iiwa[q0_index++]);
    }
  }

  systems::DiagramBuilder<T> builder;

  builder.AddSystem(std::move(owned_plant_));
  builder.AddSystem(std::move(owned_scene_graph_));

  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());

  const int num_iiwa_positions =
      plant_->num_positions(iiwa_model_.model_instance);
  DRAKE_THROW_UNLESS(num_iiwa_positions ==
                     plant_->num_velocities(iiwa_model_.model_instance));
  // Export the commanded positions via a PassThrough.
  auto iiwa_position =
      builder.template AddSystem<systems::PassThrough>(num_iiwa_positions);
  builder.ExportInput(iiwa_position->get_input_port(), "iiwa_position");
  builder.ExportOutput(iiwa_position->get_output_port(),
                       "iiwa_position_commanded");

  // Export iiwa "state" outputs.
  {
    auto demux = builder.template AddSystem<systems::Demultiplexer>(
        2 * num_iiwa_positions, num_iiwa_positions);
    builder.Connect(plant_->get_state_output_port(iiwa_model_.model_instance),
                    demux->get_input_port(0));
    builder.ExportOutput(demux->get_output_port(0), "iiwa_position_measured");
    builder.ExportOutput(demux->get_output_port(1), "iiwa_velocity_estimated");

    builder.ExportOutput(
        plant_->get_state_output_port(iiwa_model_.model_instance),
        "iiwa_state_estimated");
  }

  // Add the IIWA controller "stack".
  {
    owned_controller_plant_->Finalize();

    auto check_gains = [](const VectorX<double>& gains, int size) {
      return (gains.size() == size) && (gains.array() >= 0).all();
    };

    // Set default gains if.
    if (iiwa_kp_.size() == 0) {
      iiwa_kp_ = VectorXd::Constant(num_iiwa_positions, 100);
    }
    DRAKE_THROW_UNLESS(check_gains(iiwa_kp_, num_iiwa_positions));

    if (iiwa_kd_.size() == 0) {
      iiwa_kd_.resize(num_iiwa_positions);
      for (int i = 0; i < num_iiwa_positions; i++) {
        // Critical damping gains.
        iiwa_kd_[i] = 2 * std::sqrt(iiwa_kp_[i]);
      }
    }
    DRAKE_THROW_UNLESS(check_gains(iiwa_kd_, num_iiwa_positions));

    if (iiwa_ki_.size() == 0) {
      iiwa_ki_ = VectorXd::Constant(num_iiwa_positions, 1);
    }
    DRAKE_THROW_UNLESS(check_gains(iiwa_ki_, num_iiwa_positions));

    // Add the inverse dynamics controller.
    auto iiwa_controller = builder.template AddSystem<
        systems::controllers::InverseDynamicsController>(
        *owned_controller_plant_, iiwa_kp_, iiwa_ki_, iiwa_kd_, false);
    iiwa_controller->set_name("iiwa_controller");
    builder.Connect(plant_->get_state_output_port(iiwa_model_.model_instance),
                    iiwa_controller->get_input_port_estimated_state());

    // Add in feedforward torque.
    auto adder =
        builder.template AddSystem<systems::Adder>(2, num_iiwa_positions);
    builder.Connect(iiwa_controller->get_output_port_control(),
                    adder->get_input_port(0));
    // Use a passthrough to make the port optional.  (Will provide zero values
    // if not connected).
    auto torque_passthrough = builder.template AddSystem<systems::PassThrough>(
        Eigen::VectorXd::Zero(num_iiwa_positions));
    builder.Connect(torque_passthrough->get_output_port(),
                    adder->get_input_port(1));
    builder.ExportInput(torque_passthrough->get_input_port(),
                        "iiwa_feedforward_torque");
    builder.Connect(adder->get_output_port(), plant_->get_actuation_input_port(
                                                  iiwa_model_.model_instance));

    // Approximate desired state command from a discrete derivative of the
    // position command input port.
    auto desired_state_from_position = builder.template AddSystem<
        systems::StateInterpolatorWithDiscreteDerivative>(
            num_iiwa_positions, plant_->time_step(),
            true /* suppress_initial_transient */);
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
        manipulation::schunk_wsg::SchunkWsgPositionController>(
        manipulation::schunk_wsg::kSchunkWsgLcmStatusPeriod, wsg_kp_, wsg_kd_);
    wsg_controller->set_name("wsg_controller");

    builder.Connect(
        wsg_controller->get_generalized_force_output_port(),
        plant_->get_actuation_input_port(wsg_model_.model_instance));
    builder.Connect(plant_->get_state_output_port(wsg_model_.model_instance),
                    wsg_controller->get_state_input_port());

    builder.ExportInput(wsg_controller->get_desired_position_input_port(),
                        "wsg_position");
    builder.ExportInput(wsg_controller->get_force_limit_input_port(),
                        "wsg_force_limit");

    auto wsg_mbp_state_to_wsg_state = builder.template AddSystem(
        manipulation::schunk_wsg::MakeMultibodyStateToWsgStateSystem<double>());
    builder.Connect(plant_->get_state_output_port(wsg_model_.model_instance),
                    wsg_mbp_state_to_wsg_state->get_input_port());

    builder.ExportOutput(wsg_mbp_state_to_wsg_state->get_output_port(),
                         "wsg_state_measured");

    builder.ExportOutput(wsg_controller->get_grip_force_output_port(),
                         "wsg_force_measured");
  }

  builder.ExportOutput(plant_->get_generalized_contact_forces_output_port(
                           iiwa_model_.model_instance),
                       "iiwa_torque_external");

  {  // RGB-D Cameras
    if (render_engines.size() > 0) {
      for (auto& pair : render_engines) {
        scene_graph_->AddRenderer(pair.first, std::move(pair.second));
      }
    } else {
      scene_graph_->AddRenderer(default_renderer_name_,
                                MakeRenderEngineVtk(RenderEngineVtkParams()));
    }

    for (const auto& [name, info] : camera_information_) {
      std::string camera_name = "camera_" + name;

      const std::optional<geometry::FrameId> parent_body_id =
          plant_->GetBodyFrameIdIfExists(info.parent_frame->body().index());
      DRAKE_THROW_UNLESS(parent_body_id.has_value());
      const RigidTransform<double> X_PC =
          info.parent_frame->GetFixedPoseInBodyFrame() * info.X_PC;

      auto camera = builder.template AddSystem<systems::sensors::RgbdSensor>(
          parent_body_id.value(), X_PC, info.color_camera, info.depth_camera);
      builder.Connect(scene_graph_->get_query_output_port(),
                      camera->query_object_input_port());

      auto depth_to_cloud = builder.template AddSystem<
          perception::DepthImageToPointCloud>(
              camera->depth_camera_info(),
              systems::sensors::PixelType::kDepth16U,
              0.001f /* depth camera is in mm */,
              perception::pc_flags::kXYZs |
              perception::pc_flags::kRGBs);
      auto x_pc_system = builder.template AddSystem<
          systems::ConstantValueSource>(Value<RigidTransformd>(X_PC));
      builder.Connect(camera->color_image_output_port(),
                      depth_to_cloud->color_image_input_port());
      builder.Connect(camera->depth_image_16U_output_port(),
                      depth_to_cloud->depth_image_input_port());
      builder.Connect(x_pc_system->get_output_port(),
                      depth_to_cloud->camera_pose_input_port());

      builder.ExportOutput(camera->color_image_output_port(),
                           camera_name + "_rgb_image");
      builder.ExportOutput(camera->depth_image_16U_output_port(),
                           camera_name + "_depth_image");
      builder.ExportOutput(camera->label_image_output_port(),
                           camera_name + "_label_image");
      builder.ExportOutput(depth_to_cloud->point_cloud_output_port(),
                           camera_name + "_point_cloud");
    }
  }

  builder.ExportOutput(scene_graph_->get_query_output_port(), "query_object");

  builder.ExportOutput(scene_graph_->get_query_output_port(),
                       "geometry_query");

  builder.ExportOutput(plant_->get_contact_results_output_port(),
                       "contact_results");
  builder.ExportOutput(plant_->get_state_output_port(),
                       "plant_continuous_state");
  // TODO(SeanCurtis-TRI) It seems with the scene graph query object port
  // exported, this output port is superfluous/undesirable. This port
  // contains the FramePoseVector that connects MBP to SG. Definitely better
  // to simply rely on the query object output port.
  builder.ExportOutput(plant_->get_geometry_poses_output_port(),
                       "geometry_poses");

  builder.BuildInto(this);
}

template <typename T>
VectorX<T> ManipulationStation<T>::GetIiwaPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  return plant_->GetPositions(plant_context, iiwa_model_.model_instance);
}

template <typename T>
void ManipulationStation<T>::SetIiwaPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& q) const {
  const int num_iiwa_positions =
      plant_->num_positions(iiwa_model_.model_instance);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(q.size() == num_iiwa_positions);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetPositions(plant_context, &plant_state, iiwa_model_.model_instance,
                       q);
}

template <typename T>
VectorX<T> ManipulationStation<T>::GetIiwaVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);
  return plant_->GetVelocities(plant_context, iiwa_model_.model_instance);
}

template <typename T>
void ManipulationStation<T>::SetIiwaVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& v) const {
  const int num_iiwa_velocities =
      plant_->num_velocities(iiwa_model_.model_instance);
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(v.size() == num_iiwa_velocities);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);
  plant_->SetVelocities(plant_context, &plant_state, iiwa_model_.model_instance,
                        v);
}

template <typename T>
T ManipulationStation<T>::GetWsgPosition(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector2<T> positions =
      plant_->GetPositions(plant_context, wsg_model_.model_instance);
  return positions(1) - positions(0);
}

template <typename T>
T ManipulationStation<T>::GetWsgVelocity(
    const systems::Context<T>& station_context) const {
  const auto& plant_context =
      this->GetSubsystemContext(*plant_, station_context);

  Vector2<T> velocities =
      plant_->GetVelocities(plant_context, wsg_model_.model_instance);
  return velocities(1) - velocities(0);
}

template <typename T>
void ManipulationStation<T>::SetWsgPosition(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const T& q) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  const Vector2<T> positions(-q / 2, q / 2);
  plant_->SetPositions(plant_context, &plant_state, wsg_model_.model_instance,
                       positions);
}

template <typename T>
void ManipulationStation<T>::SetWsgVelocity(
    const drake::systems::Context<T>& station_context, systems::State<T>* state,
    const T& v) const {
  DRAKE_DEMAND(state != nullptr);
  auto& plant_context = this->GetSubsystemContext(*plant_, station_context);
  auto& plant_state = this->GetMutableSubsystemState(*plant_, state);

  const Vector2<T> velocities(-v / 2, v / 2);
  plant_->SetVelocities(plant_context, &plant_state, wsg_model_.model_instance,
                        velocities);
}

// TODO(SeanCurtis-TRI) This method does not deserve the snake_case name.
//  See https://drake.mit.edu/styleguide/cppguide.html#Function_Names
//  Deprecate and rename.
template <typename T>
std::vector<std::string> ManipulationStation<T>::get_camera_names() const {
  std::vector<std::string> names;
  names.reserve(camera_information_.size());
  for (const auto& info : camera_information_) {
    names.emplace_back(info.first);
  }
  return names;
}

template <typename T>
void ManipulationStation<T>::SetWsgGains(const double kp, const double kd) {
  DRAKE_THROW_UNLESS(!plant_->is_finalized());
  DRAKE_THROW_UNLESS(kp >= 0 && kd >= 0);
  wsg_kp_ = kp;
  wsg_kd_ = kd;
}

template <typename T>
void ManipulationStation<T>::RegisterIiwaControllerModel(
    const std::string& model_path,
    const multibody::ModelInstanceIndex iiwa_instance,
    const multibody::Frame<T>& parent_frame,
    const multibody::Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  // TODO(siyuan.feng@tri.global): We really only just need to make sure
  // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
  // from it to the world), and record that X_WP. However, the computation to
  // query X_WP given a partially constructed plant is not feasible at the
  // moment, so we are forcing the parent frame to be the world instead.
  DRAKE_THROW_UNLESS(parent_frame.name() == plant_->world_frame().name());

  iiwa_model_.model_path = model_path;
  iiwa_model_.parent_frame = &parent_frame;
  iiwa_model_.child_frame = &child_frame;
  iiwa_model_.X_PC = X_PC;

  iiwa_model_.model_instance = iiwa_instance;
}

template <typename T>
void ManipulationStation<T>::RegisterWsgControllerModel(
    const std::string& model_path,
    const multibody::ModelInstanceIndex wsg_instance,
    const multibody::Frame<T>& parent_frame,
    const multibody::Frame<T>& child_frame,
    const RigidTransform<double>& X_PC) {
  wsg_model_.model_path = model_path;
  wsg_model_.parent_frame = &parent_frame;
  wsg_model_.child_frame = &child_frame;
  wsg_model_.X_PC = X_PC;

  wsg_model_.model_instance = wsg_instance;
}

template <typename T>
void ManipulationStation<T>::RegisterRgbdSensor(
    const std::string& name, const multibody::Frame<T>& parent_frame,
    const RigidTransform<double>& X_PC,
    const geometry::render::DepthRenderCamera& depth_camera) {
  RegisterRgbdSensor(
      name, parent_frame, X_PC,
      geometry::render::ColorRenderCamera(depth_camera.core(), false),
      depth_camera);
}

template <typename T>
void ManipulationStation<T>::RegisterRgbdSensor(
    const std::string& name, const multibody::Frame<T>& parent_frame,
    const RigidTransform<double>& X_PC,
      const geometry::render::ColorRenderCamera& color_camera,
    const geometry::render::DepthRenderCamera& depth_camera) {
  CameraInformation info;
  info.parent_frame = &parent_frame;
  info.X_PC = X_PC;
  info.depth_camera = depth_camera;
  info.color_camera = color_camera;

  camera_information_[name] = info;
}

template <typename T>
std::map<std::string, RigidTransform<double>>
ManipulationStation<T>::GetStaticCameraPosesInWorld() const {
  std::map<std::string, RigidTransform<double>> static_camera_poses;

  for (const auto& info : camera_information_) {
    const auto& frame_P = *info.second.parent_frame;

    // TODO(siyuan.feng@tri.global): We really only just need to make sure
    // the parent frame is a AnchoredFrame(i.e. there is a rigid kinematic path
    // from it to the world). However, the computation to query X_WP given a
    // partially constructed plant is not feasible at the moment, so we are
    // looking for cameras that are directly attached to the world instead.
    const bool is_anchored =
        frame_P.body().index() == plant_->world_frame().body().index();
    if (is_anchored) {
      static_camera_poses.emplace(
          info.first,
          RigidTransform<double>(frame_P.GetFixedPoseInBodyFrame()) *
              info.second.X_PC);
    }
  }

  return static_camera_poses;
}

// Add default iiwa.
template <typename T>
void ManipulationStation<T>::AddDefaultIiwa(
    const IiwaCollisionModel collision_model) {
  std::string sdf_path;
  switch (collision_model) {
    case IiwaCollisionModel::kNoCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_no_collision.sdf");
      break;
    case IiwaCollisionModel::kBoxCollision:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/iiwa_description/iiwa7/"
          "iiwa7_with_box_collision.sdf");
      break;
  }
  const auto X_WI = RigidTransform<double>::Identity();
  auto iiwa_instance = internal::AddAndWeldModelFrom(
      sdf_path, "iiwa", plant_->world_frame(), "iiwa_link_0", X_WI, plant_);
  RegisterIiwaControllerModel(
      sdf_path, iiwa_instance, plant_->world_frame(),
      plant_->GetFrameByName("iiwa_link_0", iiwa_instance), X_WI);
}

// Add default wsg.
template <typename T>
void ManipulationStation<T>::AddDefaultWsg(
    const SchunkCollisionModel schunk_model) {
  std::string sdf_path;
  switch (schunk_model) {
    case SchunkCollisionModel::kBox:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/wsg_50_description/sdf"
          "/schunk_wsg_50_no_tip.sdf");
      break;
    case SchunkCollisionModel::kBoxPlusFingertipSpheres:
      sdf_path = FindResourceOrThrow(
          "drake/manipulation/models/wsg_50_description/sdf"
          "/schunk_wsg_50_with_tip.sdf");
      break;
  }
  const multibody::Frame<T>& link7 =
      plant_->GetFrameByName("iiwa_link_7", iiwa_model_.model_instance);
  const RigidTransform<double> X_7G(RollPitchYaw<double>(M_PI_2, 0, M_PI_2),
                                    Vector3d(0, 0, 0.114));
  auto wsg_instance = internal::AddAndWeldModelFrom(sdf_path, "gripper", link7,
                                                    "body", X_7G, plant_);
  RegisterWsgControllerModel(sdf_path, wsg_instance, link7,
                             plant_->GetFrameByName("body", wsg_instance),
                             X_7G);
}

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake

// TODO(russt): Support at least NONSYMBOLIC_SCALARS.  See #9573.
//   (and don't forget to include default_scalars.h)
template class ::drake::examples::manipulation_station::ManipulationStation<
    double>;
