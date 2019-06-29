#pragma once

namespace drake {
namespace solvers {

enum class SolverType {
  kCsdp,
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
  kUnrevisedLemke
};

}  // namespace solvers
}  // namespace drake
