#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/systems/estimators/luenberger_observer.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace estimators {

/// Computes the optimal observer gain, L, for the linear system defined by
///   @f[ \dot{x} = Ax + Bu + w, @f]
///   @f[ y = Cx + Du + v. @f]
/// The resulting observer is of the form
///   @f[ \dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x} - Du). @f]
/// The process noise, w, and the measurement noise, v, are assumed to be iid
/// mean-zero Gaussian.
///
/// This is a simplified form of the full Kalman filter obtained by assuming
/// that the state-covariance matrix has already converged to its steady-state
/// solution.
///
/// @param A The state-space dynamics matrix of size num_states x num_states.
/// @param C The state-space output matrix of size num_outputs x num_states.
/// @param W The process noise covariance matrix, E[ww'], of size num_states x
/// num_states.
/// @param V The measurement noise covariance matrix, E[vv'], of size num_.
/// @returns The steady-state observer gain matrix of size num_states x
/// num_outputs.
///
/// @throws std::exception if V is not positive definite.
/// @ingroup estimator_systems
/// @pydrake_mkdoc_identifier{ACWV}
///
Eigen::MatrixXd SteadyStateKalmanFilter(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V);

/// Creates a Luenberger observer system using the optimal steady-state Kalman
/// filter gain matrix, L, as described above.
///
/// @param system The LinearSystem describing the system to be observed.
/// @param W The process noise covariance matrix, E[ww'], of size num_states x
/// num_states.
/// @param V The measurement noise covariance matrix, E[vv'], of size num_.
/// @returns The constructed observer system.
///
/// @throws std::exception if V is not positive definite.
/// @ingroup estimator_systems
/// @pydrake_mkdoc_identifier{linear_system}
std::unique_ptr<LuenbergerObserver<double>> SteadyStateKalmanFilter(
    std::shared_ptr<const LinearSystem<double>> system,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V);

/// Creates a Luenberger observer system using the steady-state Kalman filter
/// observer gain.
///
/// Assuming @p system has the (continuous-time) dynamics:
///   dx/dt = f(x,u),
/// and the output:
///   y = g(x,u),
/// then the resulting observer will have the form
///   dx̂/dt = f(x̂,u) + L(y - g(x̂,u)),
/// where x̂ is the estimated state and the gain matrix, L, is designed
/// as a steady-state Kalman filter using a linearization of f(x,u) at
/// `context` as described above.
///
/// @param system The System describing the system to be observed.
/// @param context The context describing a fixed-point of the system (plus any
/// additional parameters).
/// @param W The process noise covariance matrix, E[ww'], of size num_states x
/// num_states.
/// @param V The measurement noise covariance matrix, E[vv'], of size num_.
/// @returns The constructed observer system.
///
/// @throws std::exception if V is not positive definite.
/// @ingroup estimator_systems
/// @pydrake_mkdoc_identifier{system}
std::unique_ptr<LuenbergerObserver<double>> SteadyStateKalmanFilter(
    std::shared_ptr<const System<double>> system,
    const Context<double>& context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V);

// TODO(jwnimmer-tri) Add deprecation marker on or about 2025-02-01.
/// (To be deprecated) An overload that accepts `context` by unique_ptr.
/// @exclude_from_pydrake_mkdoc{This is not bound.}
std::unique_ptr<LuenbergerObserver<double>> SteadyStateKalmanFilter(
    std::unique_ptr<System<double>> system,
    std::unique_ptr<Context<double>> context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V);

}  // namespace estimators
}  // namespace systems
}  // namespace drake
