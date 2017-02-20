#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/implicit_euler_integrator-inl.h"
#include <unsupported/Eigen/AutoDiff>

namespace drake {
namespace systems {
template class ImplicitEulerIntegrator<double>;
}  // namespace systems
}  // namespace drake

// TODO(edrumwri): correct compile bug from uncommenting line below.
// template class
// drake::systems::ImplicitEulerIntegrator<
// Eigen::AutoDiffScalar<drake::Vector1d>>;

