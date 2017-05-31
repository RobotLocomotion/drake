#include "drake/systems/analysis/half_explicit_DAE1_solver.h"
#include "drake/systems/analysis/half_explicit_DAE1_solver-inl.h"

namespace drake {
namespace systems {
template class HalfExplicitDAE1Solver<double>;
}  // namespace systems
}  // namespace drake

// TODO(edrumwri): correct compile bug from uncommenting line below.
// template class
// drake::systems::HalfExplicitDAE1Solver<
// Eigen::AutoDiffScalar<drake::Vector1d>>;

