#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/sensors/dev/rgbd_camera.h"

namespace drake {
namespace examples {
namespace manipulation_station {

/// Determines which sdf is loaded for the IIWA in the ManipulationStation.
enum class IiwaCollisionModel { kNoCollision, kBoxCollision };

/// Determines which manipulation station is simulated.
enum class Setup { kNone, kDefault, kClutterClearing };

/// @defgroup manipulation_station_systems Manipulation Station
/// @{
/// @brief Systems related to the "manipulation station" used in the <a
/// href="https://manipulation.csail.mit.edu">MIT Intelligent Robot
/// Manipulation</a> class.
/// @ingroup example_systems
/// @}

/// A system that represents the complete manipulation station, including
/// exactly one robotic arm (a Kuka IIWA LWR), one gripper (a Schunk WSG 50),
/// and anything a user might want to load into the model.
/// SetupDefaultStation() provides the setup that is used in the MIT
/// Intelligent Robot Manipulation class, which includes the supporting
/// structure for IIWA and several RGBD cameras.  Alternative Setup___()
/// methods are provided, as well.
///
/// @system{ ManipulationStation,
///   @input_port{iiwa_position}
///   @input_port{iiwa_feedforward_torque}
///   @input_port{wsg_position}
///   @input_port{wsg_force_limit},
///   @output_port{iiwa_position_commanded}
///   @output_port{iiwa_position_measured}
///   @output_port{iiwa_velocity_estimated}
///   @output_port{iiwa_state_estimated}
///   @output_port{iiwa_torque_commanded}
///   @output_port{iiwa_torque_measured}
///   @output_port{iiwa_torque_external}
///   @output_port{wsg_state_measured}
///   @output_port{wsg_force_measured}
///   @output_port{camera_[NAME]_rgb_image}
///   @output_port{camera_[NAME]_depth_image}
///   @output_port{<b style="color:orange">camera_[NAME]_label_image</b>}
///   @output_port{...}
///   @output_port{camera_[NAME]_rgb_image}
///   @output_port{camera_[NAME]_depth_image}
///   @output_port{<b style="color:orange">camera_[NAME]_label_image</b>}
///   @output_port{<b style="color:orange">pose_bundle</b>}
///   @output_port{<b style="color:orange">contact_results</b>}
///   @output_port{<b style="color:orange">plant_continuous_state</b>}
///   @output_port{<b style="color:orange">geometry_poses</b>}
/// }
///
/// Each pixel in the output image from `depth_image` is a 16bit unsigned
/// short in millimeters.
///
/// Note that outputs in <b style="color:orange">orange</b> are
/// available in the simulation, but not on the real robot.  The distinction
/// between q_measured and v_estimated is because the Kuka FRI reports
/// positions directly, but we have estimated v in our code that wraps the
/// FRI.
///
/// Consider the robot dynamics
///   M(q)vdot + C(q,v)v = τ_g(q) + τ_commanded + τ_joint_friction + τ_external,
/// where q == position, v == velocity, and τ == torque.
///
/// This model of the IIWA internal controller in the FRI software's
/// `JointImpedanceControlMode` is:
/// <pre>
///   τ_commanded = Mₑ(qₑ)vdot_desired + Cₑ(qₑ, vₑ)vₑ - τₑ_g(q) -
///                 τₑ_joint_friction + τ_feedforward
///   vdot_desired = PID(q_commanded, qₑ, v_commanded, vₑ)
/// </pre>
/// where Mₑ, Cₑ, τₑ_g, and τₑ_friction terms are now (Kuka's) estimates of the
/// true model, qₑ and vₑ are measured/estimation, and v_commanded
/// must be obtained from an online (causal) derivative of q_commanded.  The
/// result is
/// <pre>
///   M(q)vdot ≈ Mₑ(q)vdot_desired + τ_feedforward + τ_external,
/// </pre>
/// where the "approximately equal" comes from the differences due to the
/// estimated model/state.
///
/// The model implemented in this System assumes that M, C, and τ_friction
/// terms are perfect (except that they contain only a lumped mass
/// approximation of the gripper), and that the measured signals are
/// noise/bias free (e.g. q_measured = q, v_estimated = v, τ_measured =
/// τ_commanded).  What remains for τ_external is the generalized forces due
/// to contact (note that they could also include the missing contributions
/// from the gripper fingers, which the controller assumes are welded).
/// @see lcmt_iiwa_status.lcm for additional details/documentation.
///
///
/// To add objects into the environment for the robot to manipulate, use,
/// e.g.:
/// @code
/// StationSimulation<double> station;
/// Parser parser(&station.get_mutable_multibody_plant(),
///                &station.get_mutable_scene_graph());
/// parser.AddModelFromFile("my.sdf", "my_model");
/// ...
/// // coming soon -- sugar API for adding additional objects.
/// station.Finalize()
/// @endcode
/// Note that you *must* call Finalize() before you can use this class as a
/// System.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
///   - double
///
/// @ingroup manipulation_station_systems
template <typename T>
class ManipulationStation : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManipulationStation)

  /// Construct the EMPTY station model.
  ///
  /// @param time_step The time step used by MultibodyPlant<T>, and by the
  ///   discrete derivative used to approximate velocity from the position
  ///   command inputs.
  explicit ManipulationStation(double time_step = 0.002);

  /// Adds a default iiwa, wsg, two bins, and object clutter, then calls
  /// RegisterIiwaControllerModel() and RegisterWsgControllerModel() with
  /// the appropriate arguments.
  /// @note Must be called before Finalize().
  /// @note Only one of the `Setup___()` methods should be called.
  /// @param collision_model Determines which sdf is loaded for the IIWA.
  void SetupClutterClearingStation(
      IiwaCollisionModel collision_model = IiwaCollisionModel::kNoCollision);

  /// Adds a default iiwa, wsg, cupboard, and 8020 frame for the MIT
  /// Intelligent Robot Manipulation class, then calls
  /// RegisterIiwaControllerModel() and RegisterWsgControllerModel() with
  /// the appropriate arguments.
  /// @note Must be called before Finalize().
  /// @note Only one of the `Setup___()` methods should be called.
  /// @param collision_model Determines which sdf is loaded for the IIWA.
  void SetupDefaultStation(
      IiwaCollisionModel collision_model = IiwaCollisionModel::kNoCollision);

  /// Sets the default State for the chosen setup.
  /// @param context A const reference to the ManipulationStation context.
  /// @param state A pointer to the State of the ManipulationStation system.
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetDefaultState(const systems::Context<T>& station_context,
                       systems::State<T>* state) const override;

  /// Notifies the ManipulationStation that the IIWA robot model instance can
  /// be identified by @p iiwa_instance as well as necessary information to
  /// reload model for the internal controller's use. Assumes @p iiwa_instance
  /// has already been added to the MultibodyPlant.
  /// Note, the current implementation only allows @p parent_frame to be the
  /// world frame. The IIWA frame needs to directly contain @p child_frame.
  /// Only call this with custom IIWA models (i.e. not calling
  /// SetupDefaultStation()). Must be called before Finalize().
  /// @param model_path Full path to the model file.
  /// @param iiwa_instance Identifies the IIWA model.
  /// @param parent_frame Identifies frame P (the parent frame) in the
  /// MultibodyPlant that the IIWA model has been attached to.
  /// @param child_frame_name Identifies frame C (the child frame) in the IIWA
  /// model that is welded to frame P.
  /// @param X_PC Transformation between frame P and C.
  /// @throws If @p parent_frame is not the world frame.
  // TODO(siyuan.feng@tri.global): throws meaningful errors earlier here,
  // rather than in Finalize() if the arguments are inconsistent with the plant.
  // TODO(siyuan.feng@tri.global): remove the assumption that parent frame has
  // to be world.
  // TODO(siyuan.feng@tri.global): Some of these information should be
  // retrievable from the MultibodyPlant directly or MultibodyPlant should
  // provide partial tree cloning.
  void RegisterIiwaControllerModel(
      const std::string& model_path,
      const multibody::ModelInstanceIndex iiwa_instance,
      const multibody::Frame<T>& parent_frame,
      const multibody::Frame<T>& child_frame,
      const math::RigidTransform<double>& X_PC);

  /// Notifies the ManipulationStation that the WSG gripper model instance can
  /// be identified by @p wsg_instance, as well as necessary information to
  /// reload model for the internal controller's use. Assumes @p wsg_instance
  /// has already been added to the MultibodyPlant. The IIWA model needs to
  /// directly contain @p parent_frame, and the WSG model needs to directly
  /// contain @p child_frame.
  /// Only call this with custom WSG models (i.e. not calling
  /// SetupDefaultStation()). Must be called before Finalize().
  /// @param model_path Full path to the model file.
  /// @param wsg_instance Identifies the WSG model.
  /// @param parent_frame Identifies frame P (the parent frame) in the
  /// MultibodyPlant that the WSG model has been attached to. Has to be part
  /// of the IIWA model.
  /// @param child_frame Identifies frame C (the child frame) in the WSG
  /// model that is used welded to frame P.
  /// @param X_PC Transformation between frame P and C.
  // TODO(siyuan.feng@tri.global): Some of these information should be
  // retrievable from the MultibodyPlant directly or MultibodyPlant should
  // provide partial tree cloning.
  // TODO(siyuan.feng@tri.global): throws meaningful errors earlier here,
  // rather than in Finalize() if the arguments are inconsistent with the plant.
  void RegisterWsgControllerModel(
      const std::string& model_path,
      const multibody::ModelInstanceIndex wsg_instance,
      const multibody::Frame<T>& parent_frame,
      const multibody::Frame<T>& child_frame,
      const math::RigidTransform<double>& X_PC);

  /// Registers a RGBD camera. Must be called before Finalize().
  /// @param name Name for the camera.
  /// @param parent_frame The parent frame (frame P). The body that
  /// @p parent_frame is attached to must have a corresponding
  /// geometry::FrameId. Otherwise, an exception will be thrown in Finalize().
  /// @param X_PCameraBody Transformation between frame P and the camera body.
  /// see systems::sensors::dev::RgbdCamera for descriptions about how the
  /// camera body, RGB, and depth image frames are related.
  /// @param properties Properties for the RGBD camera.
  void RegisterRgbdCamera(
      const std::string& name, const multibody::Frame<T>& parent_frame,
      const math::RigidTransform<double>& X_PCameraBody,
      const geometry::dev::render::DepthCameraProperties& properties);

  // TODO(russt): Add scalar copy constructor etc once we support more
  // scalar types than T=double.  See #9573.

  /// Users *must* call Finalize() after making any additions to the
  /// multibody plant and before using this class in the Systems framework.
  /// This should be called exactly once.
  /// This assumes an IIWA and WSG have been added to the MultibodyPlant, and
  /// RegisterIiwaControllerModel() and RegisterWsgControllerModel() have been
  /// called.
  ///
  /// @see multibody::MultibodyPlant<T>::Finalize()
  void Finalize();

  /// Returns a reference to the main plant responsible for the dynamics of
  /// the robot and the environment.  This can be used to, e.g., add
  /// additional elements into the world before calling Finalize().
  const multibody::MultibodyPlant<T>& get_multibody_plant() const {
    return *plant_;
  }

  /// Returns a mutable reference to the main plant responsible for the
  /// dynamics of the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  multibody::MultibodyPlant<T>& get_mutable_multibody_plant() {
    return *plant_;
  }

  /// Returns a reference to the SceneGraph responsible for all of the geometry
  /// for the robot and the environment.  This can be used to, e.g., add
  /// additional elements into the world before calling Finalize().
  const geometry::SceneGraph<T>& get_scene_graph() const {
    return *scene_graph_;
  }

  /// Returns a mutable reference to the SceneGraph responsible for all of the
  /// geometry for the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  geometry::SceneGraph<T>& get_mutable_scene_graph() { return *scene_graph_; }

  /// Returns a const reference to the SceneGraph used for rendering
  /// camera images. Since the SceneGraph for rendering is constructed in
  /// Finalize(), this throws when called before Finalize().
  /// Note: the current implementation of the manipulation station uses a
  /// separate development version of SceneGraph for rendering (as opposed to
  /// the one returned by get_scene_graph() used for contact detection and
  /// visualization). This method will be deprecated soon.
  const geometry::dev::SceneGraph<T>& get_render_scene_graph() const {
    DRAKE_THROW_UNLESS(render_scene_graph_);
    return *render_scene_graph_;
  }

  /// Returns a mutable reference to the SceneGraph used for rendering
  /// camera images. Since the SceneGraph for rendering is constructed in
  /// Finalize(), this throws when called before Finalize().
  /// Note: the current implementation of the manipulation station uses a
  /// separate development version of SceneGraph for rendering (as opposed to
  /// the one returned by get_scene_graph() used for contact detection and
  /// visualization). This method will be deprecated soon.
  geometry::dev::SceneGraph<T>& get_mutable_render_scene_graph() {
    DRAKE_THROW_UNLESS(render_scene_graph_);
    return *render_scene_graph_;
  }

  /// Return a reference to the plant used by the inverse dynamics controller
  /// (which contains only a model of the iiwa + equivalent mass of the
  /// gripper).
  const multibody::MultibodyPlant<T>& get_controller_plant() const {
    return *owned_controller_plant_;
  }

  /// Get the number of joints in the IIWA (only -- does not include the
  /// gripper).
  int num_iiwa_joints() const { return 7; }

  /// Convenience method for getting all of the joint angles of the Kuka IIWA.
  /// This does not include the gripper.
  VectorX<T> GetIiwaPosition(const systems::Context<T>& station_context) const;

  /// Convenience method for setting all of the joint angles of the Kuka IIWA.
  /// Also sets the position history in the velocity command generator.
  /// @p q must have size num_iiwa_joints().
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetIiwaPosition(const systems::Context<T>& station_context,
                       systems::State<T>* state,
                       const Eigen::Ref<const VectorX<T>>& q) const;

  /// Convenience method for setting all of the joint angles of the Kuka IIWA.
  /// Also sets the position history in the velocity command generator.
  /// @p q must have size num_iiwa_joints().
  void SetIiwaPosition(systems::Context<T>* station_context,
                       const Eigen::Ref<const VectorX<T>>& q) const {
    SetIiwaPosition(*station_context, &station_context->get_mutable_state(), q);
  }

  DRAKE_DEPRECATED(
      "Prefer the version with the Context as the first argument."
      "This method will be deleted after 2019-04-01.")
  void SetIiwaPosition(const Eigen::Ref<const VectorX<T>>& q,
                       systems::Context<T>* station_context) const {
    SetIiwaPosition(station_context, q);
  }

  /// Convenience method for getting all of the joint velocities of the Kuka
  // IIWA.  This does not include the gripper.
  VectorX<T> GetIiwaVelocity(const systems::Context<T>& station_context) const;

  /// Convenience method for setting all of the joint velocities of the Kuka
  /// IIWA. @v must have size num_iiwa_joints().
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetIiwaVelocity(const systems::Context<T>& station_context,
                       systems::State<T>* state,
                       const Eigen::Ref<const VectorX<T>>& v) const;

  /// Convenience method for setting all of the joint velocities of the Kuka
  /// IIWA. @v must have size num_iiwa_joints().
  void SetIiwaVelocity(systems::Context<T>* station_context,
                       const Eigen::Ref<const VectorX<T>>& v) const {
    SetIiwaVelocity(*station_context, &station_context->get_mutable_state(), v);
  }

  DRAKE_DEPRECATED(
      "Prefer the version with the Context as the first argument."
      "This method will be deleted after 2019-04-01.")
  void SetIiwaVelocity(const Eigen::Ref<const VectorX<T>>& v,
                       systems::Context<T>* station_context) const {
    SetIiwaVelocity(station_context, v);
  }

  /// Convenience method for getting the position of the Schunk WSG. Note
  /// that the WSG position is the signed distance between the two fingers
  /// (not the state of the fingers individually).
  T GetWsgPosition(const systems::Context<T>& station_context) const;

  /// Convenience method for getting the velocity of the Schunk WSG.
  T GetWsgVelocity(const systems::Context<T>& station_context) const;

  /// Convenience method for setting the position of the Schunk WSG. Also
  /// sets the position history in the velocity interpolator.  Note that the
  /// WSG position is the signed distance between the two fingers (not the
  /// state of the fingers individually).
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetWsgPosition(const systems::Context<T>& station_context,
                      systems::State<T>* state, const T& q) const;

  /// Convenience method for setting the position of the Schunk WSG. Also
  /// sets the position history in the velocity interpolator.  Note that the
  /// WSG position is the signed distance between the two fingers (not the
  /// state of the fingers individually).
  void SetWsgPosition(systems::Context<T>* station_context, const T& q) const {
    SetWsgPosition(*station_context, &station_context->get_mutable_state(), q);
  }

  DRAKE_DEPRECATED(
      "Prefer the version with the Context as the first argument."
      "This method will be deleted after 2019-04-01.")
  void SetWsgPosition(const T& q, systems::Context<T>* station_context) const {
    SetWsgPosition(station_context, q);
  }

  /// Convenience method for setting the velocity of the Schunk WSG.
  /// @pre `state` must be the systems::State<T> object contained in
  /// `station_context`.
  void SetWsgVelocity(const systems::Context<T>& station_context,
                      systems::State<T>* state, const T& v) const;

  /// Convenience method for setting the velocity of the Schunk WSG.
  void SetWsgVelocity(systems::Context<T>* station_context, const T& v) const {
    SetWsgVelocity(*station_context, &station_context->get_mutable_state(), v);
  }

  DRAKE_DEPRECATED(
      "Prefer the version with the Context as the first argument."
      "This method will be deleted after 2019-04-01.")
  void SetWsgVelocity(const T& v, systems::Context<T>* station_context) const {
    SetWsgVelocity(station_context, v);
  }

  /// Returns a map from camera name to X_WCameraBody for all the static
  /// (rigidly attached to the world body) cameras that have been registered.
  std::map<std::string, math::RigidTransform<double>>
  GetStaticCameraPosesInWorld() const;

  /// Get the camera names / unique ids.
  std::vector<std::string> get_camera_names() const;

  /// Set the gains for the WSG controller.
  /// @throws exception if Finalize() has been called.
  void SetWsgGains(double kp, double kd);

  /// Set the position gains for the IIWA controller.
  /// @throws exception if Finalize() has been called.
  void SetIiwaPositionGains(const VectorX<double>& kp) {
    DRAKE_THROW_UNLESS(!plant_->is_finalized());
    iiwa_kp_ = kp;
  }

  /// Set the velocity gains for the IIWA controller.
  /// @throws exception if Finalize() has been called.
  void SetIiwaVelocityGains(const VectorX<double>& kd) {
    DRAKE_THROW_UNLESS(!plant_->is_finalized());
    iiwa_kd_ = kd;
  }

  /// Set the integral gains for the IIWA controller.
  /// @throws exception if Finalize() has been called.
  void SetIiwaIntegralGains(const VectorX<double>& ki) {
    DRAKE_THROW_UNLESS(!plant_->is_finalized());
    iiwa_ki_ = ki;
  }

 private:
  // Struct defined to store information about the how to parse and add a model.
  struct ModelInformation {
    /// This needs to have the full path. i.e. drake::FindResourceOrThrow(...)
    std::string model_path;
    multibody::ModelInstanceIndex model_instance;
    const multibody::Frame<T>* parent_frame{};
    const multibody::Frame<T>* child_frame{};
    math::RigidTransform<double> X_PC{math::RigidTransform<double>::Identity()};
  };

  struct CameraInformation {
    const multibody::Frame<T>* parent_frame{};
    math::RigidTransform<double> X_PC{math::RigidTransform<double>::Identity()};
    geometry::dev::render::DepthCameraProperties properties{
        0, 0, 0, geometry::dev::render::Fidelity::kLow, 0, 0};
  };

  // Assumes iiwa_model_info_ and wsg_model_info_ have already being populated.
  // Should only be called from Finalize().
  void MakeIiwaControllerModel();

  void AddDefaultIiwa(const IiwaCollisionModel collision_model);
  void AddDefaultWsg();

  // These are only valid until Finalize() is called.
  std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant_;
  std::unique_ptr<geometry::SceneGraph<T>> owned_scene_graph_;

  // These are valid for the lifetime of this system.
  std::unique_ptr<multibody::MultibodyPlant<T>> owned_controller_plant_;
  multibody::MultibodyPlant<T>* plant_;
  geometry::SceneGraph<T>* scene_graph_;
  // This is made in Finalize().
  geometry::dev::SceneGraph<T>* render_scene_graph_{};

  // Populated by RegisterIiwaControllerModel() and
  // RegisterWsgControllerModel().
  ModelInformation iiwa_model_;
  ModelInformation wsg_model_;

  // Registered camera related information.
  std::map<std::string, CameraInformation> camera_information_;

  // These are kp and kd gains for iiwa and wsg controllers.
  VectorX<double> iiwa_kp_;
  VectorX<double> iiwa_kd_;
  VectorX<double> iiwa_ki_;
  // TODO(siyuan.feng@tri.global): Need to tunes these better.
  double wsg_kp_{200};
  double wsg_kd_{5};

  // Represents the manipulation station to simulate. This gets set in the
  // corresponding station setup function (e.g., SetupDefaultStation()), and
  // informs how SetDefaultState() initializes the sim.
  Setup setup_{Setup::kNone};
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
