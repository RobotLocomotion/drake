#include "drake/systems/analysis/discrete_time_approximation.h"

#include <memory>

#include <unsupported/Eigen/MatrixFunctions>

namespace drake {
namespace systems {

template <typename T>
std::unique_ptr<LinearSystem<T>> DiscreteTimeApproximation(
    const LinearSystem<T>& system, double time_period) {
  // Check that the original system is continuous.
  DRAKE_THROW_UNLESS(system.IsDifferentialEquationSystem());
  // Check that the discrete time_period is greater than zero.
  DRAKE_THROW_UNLESS(time_period > 0);

  const int ns = system.num_states();
  const int ni = system.num_inputs();

  Eigen::MatrixXd M(ns + ni, ns + ni);
  M << system.A(), system.B(), Eigen::MatrixXd::Zero(ni, ns + ni);

  Eigen::MatrixXd Md = (M * time_period).exp();

  auto Ad = Md.block(0, 0, ns, ns);
  auto Bd = Md.block(0, ns, ns, ni);
  auto& Cd = system.C();
  auto& Dd = system.D();

  return std::make_unique<LinearSystem<T>>(Ad, Bd, Cd, Dd, time_period);
}

template <typename T>
std::unique_ptr<AffineSystem<T>> DiscreteTimeApproximation(
    const AffineSystem<T>& system, double time_period) {
  // Check that the original system is continuous.
  DRAKE_THROW_UNLESS(system.IsDifferentialEquationSystem());
  // Check that the discrete time_period is greater than zero.
  DRAKE_THROW_UNLESS(time_period > 0);

  const int ns = system.num_states();
  const int ni = system.num_inputs();

  Eigen::MatrixXd M(ns + ni + 1, ns + ni + 1);
  M << system.A(), system.B(), system.f0(),
      Eigen::MatrixXd::Zero(ni + 1, ns + ni + 1);

  Eigen::MatrixXd Md = (M * time_period).exp();

  auto Ad = Md.block(0, 0, ns, ns);
  auto Bd = Md.block(0, ns, ns, ni);
  auto f0d = Md.block(0, ns + ni, ns, 1);
  auto& Cd = system.C();
  auto& Dd = system.D();
  auto& y0d = system.y0();

  return std::make_unique<AffineSystem<T>>(Ad, Bd, f0d, Cd, Dd, y0d,
                                           time_period);
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
  static_cast<std::unique_ptr<LinearSystem<T>> (*)(
      const LinearSystem<T>&, double)>(&DiscreteTimeApproximation<T>),
  static_cast<std::unique_ptr<AffineSystem<T>> (*)(
      const AffineSystem<T>&, double)>(&DiscreteTimeApproximation<T>)
));

}  // namespace systems
}  // namespace drake
