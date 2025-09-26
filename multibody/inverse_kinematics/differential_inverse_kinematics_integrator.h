#pragma once

#include <vector>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {

/** A LeafSystem that integrates successive calls to
DoDifferentialInverseKinematics (which produces joint velocity commands) to
produce joint position commands.

In each evaluation, DoDifferentialInverseKinematics uses a linearization of the
robot kinematics around a nominal position. The nominal position is obtained by
either:
1. If the optional boolean (abstract-)valued input port `use_robot_state` is
   connected and set to `true`, then differential IK is computed using the
   `robot_state` input port (which must also be connected). Note: Using
   measured joint positions in a feedback loop can lead to undamped
   oscillations in the redundant joints; we hope to resolve this and are
   tracking it in #9773.
2. Otherwise, differential IK is computed using this System's internal state,
   representing the current joint position command. This is equivalent to
   integrating (open loop) the velocity commands obtained from the differential
   IK solutions.

It is also important to set the initial state of the integrator:
1. If the `robot_state` port is connected, then the initial state of the
   integrator is set to match the positions from this port (the port accepts
   the state vector with positions and velocities for easy of use with
   MultibodyPlant, but only the positions are used).
2. Otherwise, it is highly recommended that the user call SetPositions() to
   initialize the integrator state.

@system
name: DifferentialInverseKinematicsIntegrator
input_ports:
- X_AE_desired
- robot_state (optional)
- use_robot_state (optional)
output_ports:
- joint_positions
@endsystem

@ingroup manipulation_systems */
class DifferentialInverseKinematicsIntegrator
    : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DifferentialInverseKinematicsIntegrator);

  /** Constructs the system.

  @param robot A MultibodyPlant describing the robot.
  @param frame_A Reference frame (inertial or non-inertial).
  @param frame_E End-effector frame.
  @param time_step the discrete time step of the (Euler) integration.
  @param parameters Collection of various problem specific constraints and
  constants.  The `time_step` parameter will be set to @p time_step.
  @param robot_context Optional Context of the MultibodyPlant.  The position
  values of this context will be overwritten during integration; you only need
  to pass this in if the robot has any non-default parameters.  @default
  `robot.CreateDefaultContext()`.
  @param log_only_when_result_state_changes is a boolean that determines whether
  the system will log on every differential IK failure, or only when the failure
  state changes.  When the value is `true`, it will cause the system to have an
  additional discrete state variable to store the most recent
  DifferentialInverseKinematicsStatus.  Set this to `false` if you want
  IsDifferenceEquationSystem() to return `true`.

  Note: All references must remain valid for the lifetime of this system.
  @pre frame_E != frame_A.
  */
  DifferentialInverseKinematicsIntegrator(
      const MultibodyPlant<double>& robot, const Frame<double>& frame_A,
      const Frame<double>& frame_E, double time_step,
      // Note: parameters last so they could be optional in the future.
      const DifferentialInverseKinematicsParameters& parameters,
      const systems::Context<double>* robot_context = nullptr,
      bool log_only_when_result_state_changes = true);

  /** Constructs the system.

  @param robot A MultibodyPlant describing the robot.
  @param frame_E End-effector frame.
  @param time_step the discrete time step of the (Euler) integration.
  @param parameters Collection of various problem specific constraints and
  constants.  The `time_step` parameter will be set to @p time_step.
  @param robot_context Optional Context of the MultibodyPlant.  The position
  values of this context will be overwritten during integration; you only need
  to pass this in if the robot has any non-default parameters.  @default
  `robot.CreateDefaultContext()`.
  @param log_only_when_result_state_changes is a boolean that determines whether
  the system will log on every differential IK failure, or only when the failure
  state changes.  When the value is `true`, it will cause the system to have an
  additional discrete state variable to store the most recent
  DifferentialInverseKinematicsStatus.  Set this to `false` if you want
  IsDifferenceEquationSystem() to return `true`.

  In this overload, the reference frame, A, is taken to be the world frame.

  Note: All references must remain valid for the lifetime of this system.
  @pre frame_E != robot.world_frame().
  */
  DifferentialInverseKinematicsIntegrator(
      const MultibodyPlant<double>& robot, const Frame<double>& frame_E,
      double time_step,
      // Note: parameters last so they could be optional in the future.
      const DifferentialInverseKinematicsParameters& parameters,
      const systems::Context<double>* robot_context = nullptr,
      bool log_only_when_result_state_changes = true);

  /** Sets the joint positions, which are stored as state in the context. It is
  recommended that the user calls this method to initialize the position
  commands to match the initial positions of the robot. */
  void SetPositions(systems::Context<double>* context,
                    const Eigen::Ref<const Eigen::VectorXd>& positions) const;

  /** Provides X_AE as a function of the joint position set in `context`. */
  math::RigidTransformd ForwardKinematics(
      const systems::Context<double>& context) const;

  /** Returns a const reference to the differential IK parameters owned by this
  system. */
  const DifferentialInverseKinematicsParameters& get_parameters() const;

  /** Returns a mutable reference to the  differential IK parameters owned by
  this system. */
  DifferentialInverseKinematicsParameters& get_mutable_parameters();

 private:
  // Updates the position in the cached Context for the robot to match the
  // internal state of the integrator.
  void UpdateRobotContext(const systems::Context<double>& context,
                          systems::Context<double>* robot_context) const;

  // Calls DoDifferentialInverseKinematics and performs one integration step.
  systems::EventStatus Integrate(
      const systems::Context<double>& context,
      systems::DiscreteValues<double>* discrete_state) const;

  // Outputs the current position value.
  void CopyPositionsOut(const systems::Context<double>& context,
                        systems::BasicVector<double>* output) const;

  // If the state input port is connected, then this method sets the integrator
  // state to match the positions on the input port.
  systems::EventStatus Initialize(
      const systems::Context<double>& context,
      systems::DiscreteValues<double>* values) const;

  const MultibodyPlant<double>& robot_;
  const Frame<double>& frame_A_;
  const Frame<double>& frame_E_;
  DifferentialInverseKinematicsParameters parameters_;
  const double time_step_{0.0};
  const systems::CacheEntry* robot_context_cache_entry_{};
  systems::InputPortIndex X_AE_desired_index_{};
  systems::InputPortIndex X_WE_desired_index_{};
  systems::InputPortIndex robot_state_index_{};
  systems::InputPortIndex use_robot_state_index_{};
};

}  // namespace multibody
}  // namespace drake
