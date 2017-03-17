#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator-inl.h"

#include <unsupported/Eigen/AutoDiff>

namespace drake {
namespace systems {
template class RungeKutta3Integrator<double>;
}  // namespace systems
}  // namespace drake

// TODO(edrumwri): correct compile bug from uncommenting line below.
// template class
// drake::systems::RungeKutta3Integrator<
// Eigen::AutoDiffScalar<drake::Vector1d>>;

