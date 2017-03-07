#include "drake/solvers/moby_lcp_solver.h"
#include "drake/solvers/moby_lcp_solver-inl.h"
#include <unsupported/Eigen/AutoDiff>

namespace drake {
namespace solvers {
template class MobyLCPSolver<double>;
}  // namespace solvers
}  // namespace drake

// TODO(edrumwri): correct compile bug from uncommenting line below.
template class
    drake::solvers::MobyLCPSolver<Eigen::AutoDiffScalar<drake::Vector1d>>;

