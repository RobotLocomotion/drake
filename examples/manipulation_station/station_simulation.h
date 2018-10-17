#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace manipulation_station {

/// @defgroup manipulation_station_systems Manipulation Station
/// @{
/// @brief Systems related to the "manipulation station" used in the <a
/// href="https://manipulation.csail.mit.edu">MIT Intelligent Robot
/// Manipulation</a> class.
/// @ingroup example_systems
/// @}

// TODO(russt): Add WSG I/O and helper methods for setting the context.
// TODO(russt): Add camera outputs.
// TODO(russt): Refactor kuka+wsg subset into a reusable component during the
//              upcoming kuka_iiwa directory cleanup.
/// A system that represents the complete manipulation station, including the
/// robotic arm (a Kuka IIWA LWR), the gripper (a Schunk WSG 50), the
/// externally mounted sensors, and the structure that it is mounted into.
/// @{
///
/// @system{ StationSimulation,
///   @input_port{iiwa_position}
///   @input_port{iiwa_feedforward_torque},
///   @output_port{iiwa_position_commanded}
///   @output_port{iiwa_position_measured}
///   @output_port{iiwa_velocity_estimated}
///   @output_port{iiwa_state_estimated}
///   @output_port{iiwa_torque_commanded}
///   @output_port{iiwa_torque_measured}
///   @output_port{iiwa_torque_external}
///   @output_port{<b style="color:orange">pose_bundle</b>} }
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
/// AddModelFromSdfFile("my.sdf", "my_model",
///                     station.get_mutable_multibody_plant(),
///                     station.get_mutable_scene_graph());
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
/// @}
template <typename T>
class StationSimulation : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StationSimulation)

  /// Construct the station model with @p time_step as the time step used by
  /// MultibodyPlant<T>, and by the discrete derivative used to approximate
  /// velocity from the position command inputs.
  explicit StationSimulation(double time_step = 0.002);

  // TODO(russt): Add scalar copy constructor etc once we support more
  // scalar types than T=double.  See #9573.

  /// Users *must* call Finalize() after making any additions to the
  /// multibody plant and before using this class in the Systems framework.
  /// This should be called exactly once.
  ///
  /// @see multibody::multibody_plant::MultibodyPlant<T>::Finalize()
  void Finalize();

  /// Return a mutable reference to the main plant responsible for the
  /// dynamics of the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  multibody::multibody_plant::MultibodyPlant<T>& get_mutable_multibody_plant() {
    return *plant_;
  }

  /// Return a mutable reference to the SceneGraph responsible for all of the
  /// geometry for the robot and the environment.  This can be used to, e.g.,
  /// add additional elements into the world before calling Finalize().
  geometry::SceneGraph<T>& get_mutable_scene_graph() {
    return *scene_graph_;
  }

  /// Return a reference to the plant used by the inverse dynamics controller
  /// (which contains only a model of the iiwa + equivalent mass of the
  /// gripper).
  const multibody::multibody_plant::MultibodyPlant<T>& get_controller_plant()
      const {
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
  void SetIiwaPosition(const Eigen::Ref<const VectorX<T>>& q,
                        systems::Context<T>* station_context) const;

  /// Convenience method for getting all of the joint velocities of the Kuka
  // IIWA.  This does not include the gripper.
  VectorX<T> GetIiwaVelocity(
      const systems::Context<T>& station_context) const;

  /// Convenience method for setting all of the joint velocities of the Kuka
  /// IIWA. @v must have size num_iiwa_joints().
  void SetIiwaVelocity(const Eigen::Ref<const VectorX<T>>& v,
                         systems::Context<T>* station_context) const;

  // TODO(russt): Implement SetIiwaPIDGains(...).

 private:
  // These are only valid until Finalize() is called.
  std::unique_ptr<multibody::multibody_plant::MultibodyPlant<T>> owned_plant_;
  std::unique_ptr<geometry::SceneGraph<T>> owned_scene_graph_;

  // These are valid for the lifetime of this system.
  std::unique_ptr<multibody::multibody_plant::MultibodyPlant<T>>
      owned_controller_plant_;
  multibody::multibody_plant::MultibodyPlant<T>* plant_;
  geometry::SceneGraph<T>* scene_graph_;

  multibody::ModelInstanceIndex iiwa_model_;
  multibody::ModelInstanceIndex wsg_model_;
};

}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
