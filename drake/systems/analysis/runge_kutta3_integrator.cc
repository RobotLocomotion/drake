#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator-inl.h"

#include <unsupported/Eigen/AutoDiff>

namespace drake {
namespace systems {
template class RungeKutta3Integrator<double>;
template class RungeKutta3Integrator<AutoDiffXd>;
}  // namespace systems
}  // namespace drake
