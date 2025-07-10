#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/discrete_time_integrator.h"

namespace drake {
namespace multibody {

/** Differential Inverse Kinematics controller that tracks desired poses /
velocities for multiple operational points. The controller tracks a nominal
posture to resolve nullspace.

The diagram implements an open loop controller where the previous commanded
position is fed back as input estimated position to the the diagram.

TODO(Aditya.Bhat): Add port switch to switch between open and closed loop
control.

@pre The initial position must be set using the
`set_initial_position(systems::Context<double>* context, const
Eigen::Ref<const Eigen::VectorXd>& value)` so that the open loop controller can
start from the correct robot state.

@system
name: DifferentialInverseKinematicsController
input_ports:
- estimated_state
- desired_poses
- nominal_posture
output_ports:
- commanded_position
- commanded_velocity
@endsystem

@tparam_default_scalar
@ingroup control_systems */
class DifferentialInverseKinematicsController final
    : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DifferentialInverseKinematicsController);

  /** Constructs a DifferentialInverseKinematicsController.
  @param differential_inverse_kinematics a DifferentialInverseKinematicsSystem
                                         leaf system.
  @param planar_rotation_dof_indices the indices of commanded position output
                                     that will be wrapped to ±π.

  @warning a DifferentialInverseKinematicsSystem leaf system may only be added
  to at most one controller. Multiple controller instances cannot share the same
  leaf system. */
  DifferentialInverseKinematicsController(
      std::shared_ptr<DifferentialInverseKinematicsSystem>
          differential_inverse_kinematics,
      const std::vector<int>& planar_rotation_dof_indices);

  ~DifferentialInverseKinematicsController() final;

  /** Sets the integral part of the DiscreteTimeIntegrator to `value`. `value`
  has the dimension of the full `plant.num_positions()`; non-active dofs will be
  ignored. This in effect sets the initial position that is fed to the
  DifferentialInverseKinematicsSystem leaf system. */
  void set_initial_position(
      systems::Context<double>* context,
      const Eigen::Ref<const Eigen::VectorXd>& value) const;

  /** Sets the default state of the controller. */
  void SetDefaultState(const systems::Context<double>& context,
                       systems::State<double>* state) const final;

  /** Sets the random state of the controller. */
  void SetRandomState(const systems::Context<double>& context,
                      systems::State<double>* state,
                      RandomGenerator* generator) const final;

  const DifferentialInverseKinematicsSystem& differential_inverse_kinematics() {
    return *differential_inverse_kinematics_;
  }

  DifferentialInverseKinematicsSystem&
  get_mutable_differential_inverse_kinematics() {
    return *differential_inverse_kinematics_;
  }

 private:
  void set_state_to_nan(const systems::Context<double>& context,
                        systems::State<double>* state) const;

  DifferentialInverseKinematicsSystem* differential_inverse_kinematics_{
      nullptr};
  systems::DiscreteTimeIntegrator<double>* discrete_time_integrator_{nullptr};
};

}  // namespace multibody
}  // namespace drake
