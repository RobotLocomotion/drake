#pragma once

#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {

struct LinearQuadraticRegulatorResult {
  Eigen::MatrixXd S;
  Eigen::MatrixXd K;
};

/// Computes the optimal feedback controller, u=-Kx
///
///   @f[ \dot{x} = Ax + Bu @f]
///   @f[ \min_u \int_0^T x'Qx + u'Ru + 2x'Nu dt @f]
///
/// @param A The state-space dynamics matrix of size num_states x num_states.
/// @param B The state-space input matrix of size num_states x num_inupts.
/// @param Q A symmetric positive semi-definite cost matrix of size num_states x
/// num_states.
/// @param R A symmetric positive definite cost matrix of size num_inputs x
/// num_inputs.
/// @param N A cost matrix of size num_states x num_inputs.
/// @returns A structure that contains the optimal feedback gain K and the
/// quadratic cost term S. The optimal feedback control is u = -Kx;
///
/// @throws std::runtime_error if R is not positive definite.
///
LinearQuadraticRegulatorResult LinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero());

/// Creates a system that implements the optimal time-invariant linear quadratic
/// regulator (LQR):
///
///   @f[ \min_u \int_0^T x'Qx + u'Ru dt @f]
///
/// @param system The System to be controlled.
/// @param Q A symmetric positive semi-definite cost matrix of size num_states x
/// num_states.
/// @param R A symmetric positive definite cost matrix of size num_inputs x
/// num_inputs.
/// @param N A cost matrix of size num_states x num_inputs.
/// @returns A system implementing the optimal controller in the original system
/// coordinates.
///
/// @throws std::runtime_error if R is not positive definite.
///
std::unique_ptr<LinearSystem<double>> LinearQuadraticRegulator(
    const LinearSystem<double>& system,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero());

/// Linearizes the System around the specified Context, computes the optimal
/// time-invariant linear quadratic regulator (LQR), and returns a System which
/// implements that regulator in the original System's coordinates.
///
/// @f[ \min_u \int_0^T (x-x_0)'Q(x-x_0) + (u-u_0)'R(u-u_0) dt @f]
/// where @f$ x_0 @f$ is the nominal state and @f$ u_0 @f$ is the nominal input.
///
/// @param system The System to be controlled.
/// @param context Defines the desired state and control input to regulate the
/// system to.  Note that this state/input must be an equilibrium point of the
/// system.  See drake::systems::Linearize for more details.
/// @param Q A symmetric positive semi-definite cost matrix of size num_states x
/// num_states.
/// @param R A symmetric positive definite cost matrix of size num_inputs x
/// num_inputs.
/// @param N A cost matrix of size num_states x num_inputs.
/// @returns A system implementing the optimal controller in the original system
/// coordinates.
///
/// @throws std::runtime_error if R is not positive definite.
///
std::unique_ptr<AffineSystem<double>> LinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero());

}  // namespace systems
}  // namespace drake
