#include "drake/systems/analysis/runge_kutta5_integrator.h"
#include "drake/systems/analysis/runge_kutta5_integrator-inl.h"

#include "drake/common/autodiff.h"

namespace drake {
namespace systems {
template class RungeKutta5Integrator<double>;
template class RungeKutta5Integrator<AutoDiffXd>;
}  // namespace systems
}  // namespace drake
