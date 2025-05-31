#include "drake/systems/estimators/kalman_filter.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/math/continuous_algebraic_riccati_equation.h"
#include "drake/math/discrete_algebraic_riccati_equation.h"
#include "drake/systems/estimators/luenberger_observer.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace estimators {

Eigen::MatrixXd SteadyStateKalmanFilter(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V) {
  const auto& P = math::ContinuousAlgebraicRiccatiEquation(A.transpose(),
                                                           C.transpose(), W, V);
  return P * C.transpose() * V.inverse();
}

Eigen::MatrixXd DiscreteTimeSteadyStateKalmanFilter(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V) {
  // Kalman filter dynamics:
  // K = P̂C'(CP̂C' + V)⁻¹
  // x̂[n|n] = x̂[n|n-1] + K(y - ŷ)
  // x̂[n+1|n] = Ax̂[n|n] + Bu[n]
  // Steady state dynamics:
  // x̂[n+1|n] = Ax̂[n|n-1] + Bu[n] + L(y - ŷ)
  // Therefore, L = AK = APC'(CPC' + V)⁻¹
  const auto& P = math::DiscreteAlgebraicRiccatiEquation(A.transpose(),
                                                         C.transpose(), W, V);
  return (C * P * C.transpose() + V)
      .llt()
      .solve(C * P * A.transpose())
      .transpose();
}

std::unique_ptr<LuenbergerObserver<double>> SteadyStateKalmanFilter(
    std::shared_ptr<const LinearSystem<double>> system,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V) {
  const bool is_continuous = system->IsDifferentialEquationSystem();
  const Eigen::MatrixXd L =
      is_continuous
          ? SteadyStateKalmanFilter(system->A(), system->C(), W, V)
          : DiscreteTimeSteadyStateKalmanFilter(system->A(), system->C(), W, V);

  return std::make_unique<LuenbergerObserver<double>>(
      std::move(system), *system->CreateDefaultContext(), L);
}

std::unique_ptr<LuenbergerObserver<double>> SteadyStateKalmanFilter(
    std::shared_ptr<const System<double>> system,
    const Context<double>& context, const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V) {
  const bool is_continuous = system->IsDifferentialEquationSystem();
  DRAKE_DEMAND((is_continuous ? context.get_continuous_state_vector()
                              : context.get_discrete_state_vector())
                   .size() > 0);  // Otherwise, I don't need an estimator.
  DRAKE_DEMAND(system->num_output_ports() ==
               1);  // Need measurements to estimate state.

  // TODO(russt): Demand time-invariant once we can.

  std::unique_ptr<LinearSystem<double>> linear_system =
      Linearize(*system, context);

  const Eigen::MatrixXd L =
      is_continuous ? SteadyStateKalmanFilter(linear_system->A(),
                                              linear_system->C(), W, V)
                    : DiscreteTimeSteadyStateKalmanFilter(
                          linear_system->A(), linear_system->C(), W, V);

  return std::make_unique<LuenbergerObserver<double>>(std::move(system),
                                                      context, L);
}

}  // namespace estimators
}  // namespace systems
}  // namespace drake
