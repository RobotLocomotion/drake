#include "runge_kutta3_integrator-inl.h"
#include <unsupported/Eigen/AutoDiff>

template class DRAKE_EXPORT drake::systems::RungeKutta3Integrator<double>;

// TODO(edrumwri): uncomment this when suport for AutoDiff NaN/Inf checks
// available. See issue #4005.
// template class DRAKE_EXPORT
//drake::systems::RungeKutta3Integrator<Eigen::AutoDiffScalar<drake::Vector1d>>;

