#pragma once

#include "drake/systems/framework/primitives/linear_system.h"

namespace drake {
namespace systems {

/// Linearizes the System around the specified Context, computes the optimal
/// time-invariant linear quadratic regulator (LQR), and returns a system which
/// implements that regulator in the original System's coordinates.
///
/// @f[ \min_u \int_0^T (x-x_0)'Q(x-x_0) + (u-u_0)'R(u-u_0) dt @f]
/// where @f$ x_0 @f$ is the nominal state and @f$ u_0 @f$ is the nominal input.
///
/// @param system The System to be controlled.
/// @param context Defines the desired state and control input to regulate the
/// system to.  Note that this state/input must be an equilibrium point of the
/// system.  See drake::systems::Linearize for more details.
/// @param Q A symmetric positive semi-definite matrix of size num_states x
/// num_states.
/// @param R A symmetric positive definite matrix of size num_inputs x
/// num_inputs.
/// @returns A system implementing the optimal controller in the original system
/// coordinates.
///
/// @throws std::runtime_error if R is not positive definite.
///

std::unique_ptr<systems::AffineSystem<double>> TimeInvariantLqr(
    const System<double>& system, const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R);

}  // namespace systems
}  // namespace drake
