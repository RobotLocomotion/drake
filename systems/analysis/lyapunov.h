#pragma once

#include <functional>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace analysis {

/// Sets up a linear program to search for the coefficients of a
/// Lyapunov function that satisfies the Lyapunov conditions at a set
/// of sample points.
///   ∀xᵢ, V(xᵢ) ≥ 0,
///   ∀xᵢ, V̇(xᵢ) = ∂V/∂x f(xᵢ) ≤ 0.
/// In order to provide boundary conditions to the problem, and improve
/// numerical conditioning, we additionally impose the constraint
///   V(x₀) = 0,
/// and add an objective that pushes V̇(xᵢ) towards -1 (time-to-go):
///   min ∑ |V̇(xᵢ) + 1|.
///
/// For background, and a description of this algorithm, see
/// http://underactuated.csail.mit.edu/underactuated.html?chapter=lyapunov .
/// It currently requires that the system to be optimized has only continuous
/// state and it is assumed to be time invariant.
///
/// @param system to be verified.  We currently require that the system has
/// only continuous state, and it is assumed to be time invariant.  Unlike
/// many analysis algorithms, the system does *not* need to support conversion
/// to other ScalarTypes (double is sufficient).
///
/// @param context is used only to specify any parameters of the system, and to
/// fix any input ports.  The system/context must have all inputs assigned.
///
/// @param basis_functions must define an AutoDiffXd function that takes the
/// state vector as an input argument and returns the vector of values of the
/// basis functions at that state.  The Lyapunov function will then have the
/// form
///   V(x) = ∑ pᵢ φᵢ(x),
/// where `p` is the vector to be solved for and `φ(x)` is the vector of
/// basis function evaluations returned by this function.
///
/// @param state_samples is a list of sample states (one per column) at which
/// to apply the optimization constraints and the objective.
///
/// @param V_zero_state is a particular state, x₀, where we impose the
/// condition: V(x₀) = 0.
///
/// @return params the VectorXd of parameters, p, that satisfies the Lyapunov
/// conditions described above.  The resulting Lyapunov function is
///   V(x) = ∑ pᵢ φᵢ(x),
///
/// @ingroup analysis
Eigen::VectorXd SampleBasedLyapunovAnalysis(
    const System<double>& system, const Context<double>& context,
    const std::function<VectorX<AutoDiffXd>(const VectorX<AutoDiffXd>& state)>&
        basis_functions,
    const Eigen::Ref<const Eigen::MatrixXd>& state_samples,
    const Eigen::Ref<const Eigen::VectorXd>& V_zero_state);

}  // namespace analysis
}  // namespace systems
}  // namespace drake
