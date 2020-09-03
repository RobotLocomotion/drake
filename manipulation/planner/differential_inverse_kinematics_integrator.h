#pragma once

#include <vector>

#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace planner {

/** A LeafSystem which uses DoDifferentialInverseKinematics to produce joint
position commands.

Rather than calling DoDifferentialInverseKinematics on the current measured
positions of the robot, this System maintains its own internal state and
integrates successive velocity commands open loop.  Using measured joint
positions in a feedback loop can lead to undamped oscillations in the redundant
joints; we hope to resolve this and are tracking it in #9773.

Note: It is highly recommended that the user calls `SetPosition()` once to
initialize the position commands to match the initial positions of the robot.

@system
name: DifferentialInverseKinematicsIntegrator
input_ports:
- X_WE_desired
output_ports:
- joint_positions
@endsystem

@ingroup manipulation_systems */
class DifferentialInverseKinematicsIntegrator
    : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DifferentialInverseKinematicsIntegrator)

  /** Constructs the system.

  @param robot A MultibodyPlant describing the robot.
  @param frame_E End-effector frame.
  @param time_step the discrete time step of the (Euler) integration.
  @param parameters Collection of various problem specific constraints and
  constants.  The `timestep` parameter will be set to @p time_step.
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
  */
  DifferentialInverseKinematicsIntegrator(
      const multibody::MultibodyPlant<double>& robot,
      const multibody::Frame<double>& frame_E, double time_step,
      // Note: parameters last so they could be optional in the future.
      const DifferentialInverseKinematicsParameters& parameters,
      const systems::Context<double>* robot_context = nullptr,
      bool log_only_when_result_state_changes = true);

  /** Sets the joint positions, which are stored as state in the context. It is
  recommended that the user calls this method to initialize the position
  commands to match the initial positions of the robot. */
  void SetPositions(systems::Context<double>* context,
                    const Eigen::Ref<const Eigen::VectorXd>& positions) const;

  /** Provides X_WE as a function of the joint position set in `context`. */
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
  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>& events,
      systems::DiscreteValues<double>* discrete_state) const override;

  // Outputs the current position value.
  void CopyPositionsOut(const systems::Context<double>& context,
                        systems::BasicVector<double>* output) const;

  const multibody::MultibodyPlant<double>& robot_;
  const multibody::Frame<double>& frame_E_;
  DifferentialInverseKinematicsParameters parameters_;
  const double time_step_{0.0};
  const systems::CacheEntry* robot_context_cache_entry_{};
};

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
