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
  kOsqp,
  kSnopt,
  kScs,
};

}  // namespace solvers
}  // namespace drake
