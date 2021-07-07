#pragma once

#include <memory>

#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace controllers {

struct LinearQuadraticRegulatorResult {
  Eigen::MatrixXd K;
  Eigen::MatrixXd S;
};

/// Computes the optimal feedback controller, u=-Kx, and the optimal
/// cost-to-go J = x'Sx for the problem:
///
///   @f[ \dot{x} = Ax + Bu @f]
///   @f[ \min_u \int_0^\infty x'Qx + u'Ru + 2x'Nu dt @f]
///
/// @param A The state-space dynamics matrix of size num_states x num_states.
/// @param B The state-space input matrix of size num_states x num_inputs.
/// @param Q A symmetric positive semi-definite cost matrix of size num_states x
/// num_states.
/// @param R A symmetric positive definite cost matrix of size num_inputs x
/// num_inputs.
/// @param N A cost matrix of size num_states x num_inputs. If the matrix is
/// zero-sized, N will be treated as a num_states x num_inputs zero matrix.
/// @returns A structure that contains the optimal feedback gain K and the
/// quadratic cost term S. The optimal feedback control is u = -Kx;
///
/// @throws std::exception if R is not positive definite.
/// @ingroup control
///
LinearQuadraticRegulatorResult LinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero());

// TODO(russt): Consider implementing the optional N argument as in the
// continuous-time formulation.
/// Computes the optimal feedback controller, u=-Kx, and the optimal
/// cost-to-go J = x'Sx for the problem:
///
///   @f[ x[n+1] = Ax[n] + Bu[n] @f]
///   @f[ \min_u \sum_0^\infty x'Qx + u'Ru @f]
///
/// @param A The state-space dynamics matrix of size num_states x num_states.
/// @param B The state-space input matrix of size num_states x num_inputs.
/// @param Q A symmetric positive semi-definite cost matrix of size num_states x
/// num_states.
/// @param R A symmetric positive definite cost matrix of size num_inputs x
/// num_inputs.
/// @returns A structure that contains the optimal feedback gain K and the
/// quadratic cost term S. The optimal feedback control is u = -Kx;
///
/// @throws std::exception if R is not positive definite.
/// @ingroup control
LinearQuadraticRegulatorResult DiscreteTimeLinearQuadraticRegulator(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R);

/// Creates a system that implements the optimal time-invariant linear quadratic
/// regulator (LQR).  If @p system is a continuous-time system, then solves
/// the continuous-time LQR problem:
///
///   @f[ \min_u \int_0^\infty x^T(t)Qx(t) + u^T(t)Ru(t) dt. @f]
///
/// If @p system is a discrete-time system, then solves the discrete-time LQR
/// problem:
///
///   @f[ \min_u \sum_0^\infty x^T[n]Qx[n] + u^T[n]Ru[n]. @f]
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
/// @throws std::exception if R is not positive definite.
/// @ingroup control_systems
///
std::unique_ptr<LinearSystem<double>> LinearQuadraticRegulator(
    const LinearSystem<double>& system,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero());

/// Linearizes the System around the specified Context, computes the optimal
/// time-invariant linear quadratic regulator (LQR), and returns a System which
/// implements that regulator in the original System's coordinates.  If
/// @p system is a continuous-time system, then solves
/// the continuous-time LQR problem:
///
///   @f[ \min_u \int_0^\infty (x-x_0)^TQ(x-x_0) + (u-u_0)^TR(u-u_0) dt. @f]
///
/// If @p system is a discrete-time system, then solves the discrete-time LQR
/// problem:
///
///   @f[ \min_u \sum_0^\infty (x-x_0)^TQ(x-x_0) + (u-u_0)^TR(u-u_0), @f]
///
/// where @f$ x_0 @f$ is the nominal state and @f$ u_0 @f$ is the nominal input.
/// The system is considered discrete if it has a single discrete state
/// vector and a single unique periodic update event declared.
///
/// @param system The System to be controlled.
/// @param context Defines the desired state and control input to regulate the
/// system to.  Note that this state/input must be an equilibrium point of the
/// system.  See drake::systems::Linearize for more details.
/// @param Q A symmetric positive semi-definite cost matrix of size num_states x
/// num_states.
/// @param R A symmetric positive definite cost matrix of size num_inputs x
/// num_inputs.
/// @param N A cost matrix of size num_states x num_inputs.  If the matrix is
/// zero-sized, N will be treated as a num_states x num_inputs zero matrix.
/// @param int_port_index The index of the input port to linearize around.
/// @returns A system implementing the optimal controller in the original system
/// coordinates.
///
/// @throws std::exception if R is not positive definite.
/// @ingroup control_systems
/// @see drake::systems::Linearize()
///
std::unique_ptr<AffineSystem<double>> LinearQuadraticRegulator(
    const System<double>& system, const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N =
        Eigen::Matrix<double, 0, 0>::Zero(),
    int input_port_index = 0);

}  // namespace controllers
}  // namespace systems
}  // namespace drake
