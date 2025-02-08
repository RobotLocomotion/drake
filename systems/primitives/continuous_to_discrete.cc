#include "drake/systems/primitives/continuous_to_discrete.h"

#include <unsupported/Eigen/MatrixFunctions>

namespace drake {
namespace systems {

using Eigen::MatrixXd;

std::unique_ptr<LinearSystem<double>> ContinuousToDiscrete(
    const LinearSystem<double>& system, double time_period) {
  // check that the original system is continuous
  DRAKE_THROW_UNLESS(system.time_period() == 0);
  // check that the discrete time_period is greater than zero
  DRAKE_THROW_UNLESS(time_period > 0);

  const int ns = system.num_states();
  const int ni = system.num_inputs();

  MatrixXd M(ns + ni, ns + ni);
  M << system.A(), system.B(), MatrixXd::Zero(ni, ns + ni);

  MatrixXd Md = (M * time_period).exp();

  auto Ad = Md.block(0, 0, ns, ns);
  auto Bd = Md.block(0, ns, ns, ni);
  auto& Cd = system.C();
  auto& Dd = system.D();

  return std::make_unique<LinearSystem<double>>(Ad, Bd, Cd, Dd, time_period);
}

std::unique_ptr<AffineSystem<double>> ContinuousToDiscrete(
    const AffineSystem<double>& system, double time_period) {
  // check that the original system is continuous
  DRAKE_THROW_UNLESS(system.time_period() == 0);
  // check that the discrete time_period is greater than zero
  DRAKE_THROW_UNLESS(time_period > 0);

  const int ns = system.num_states();
  const int ni = system.num_inputs();

  MatrixXd M(ns + ni + 1, ns + ni + 1);
  M << system.A(), system.B(), system.f0(), MatrixXd::Zero(ni + 1, ns + ni + 1);

  MatrixXd Md = (M * time_period).exp();

  auto Ad = Md.block(0, 0, ns, ns);
  auto Bd = Md.block(0, ns, ns, ni);
  auto f0d = Md.block(0, ns + ni, ns, 1);
  auto& Cd = system.C();
  auto& Dd = system.D();
  auto& y0d = system.y0();

  return std::make_unique<AffineSystem<double>>(Ad, Bd, f0d, Cd, Dd, y0d,
                                                time_period);
}

}  // namespace systems
}  // namespace drake
