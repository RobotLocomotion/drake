#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator-inl.h"
#include <unsupported/Eigen/AutoDiff>

template class DRAKE_EXPORT drake::systems::RungeKutta3Integrator<double>;

// TODO(edrumwri): correct compile bug from uncommenting line below.
// template class DRAKE_EXPORT
// drake::systems::RungeKutta3Integrator<
// Eigen::AutoDiffScalar<drake::Vector1d>>;

