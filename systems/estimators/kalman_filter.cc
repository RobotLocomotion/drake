#include "drake/systems/estimators/kalman_filter.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/math/continuous_algebraic_riccati_equation.h"
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
  const auto& P =
      math::ContinuousAlgebraicRiccatiEquation(
          A.transpose(), C.transpose(), W, V);
  return P * C.transpose() * V.inverse();
}

std::unique_ptr<LuenbergerObserver<double>> SteadyStateKalmanFilter(
    std::unique_ptr<LinearSystem<double>> system,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V) {
  const Eigen::MatrixXd L =
      SteadyStateKalmanFilter(system->A(), system->C(), W, V);

  return std::make_unique<LuenbergerObserver<double>>(
      std::move(system), *system->CreateDefaultContext(), L);
}

std::unique_ptr<LuenbergerObserver<double>> SteadyStateKalmanFilter(
    std::unique_ptr<System<double>> system,
    std::unique_ptr<Context<double>> context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V) {
  DRAKE_DEMAND(context->get_continuous_state_vector().size() >
               0);  // Otherwise, I don't need an estimator.
  DRAKE_DEMAND(system->num_output_ports() ==
               1);  // Need measurements to estimate state.

  // TODO(russt): Demand time-invariant once we can.
  // TODO(russt): Check continuous-time (only).

  auto linear_system = Linearize(*system, *context);

  const Eigen::MatrixXd L =
      SteadyStateKalmanFilter(linear_system->A(), linear_system->C(), W, V);

  return std::make_unique<LuenbergerObserver<double>>(std::move(system),
                                                      *context, L);
}

}  // namespace estimators
}  // namespace systems
}  // namespace drake
