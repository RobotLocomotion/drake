#pragma once

namespace drake {
namespace solvers {

enum class SolverType {
  kDReal,
  kEqualityConstrainedQP,
  kGurobi,
  kIpopt,
  kLinearSystem,
  kMobyLCP,
  kMosek,
  kNlopt,
  kSnopt,
};

}  // namespace solvers
}  // namespace drake
