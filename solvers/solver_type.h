#pragma once

namespace drake {
namespace solvers {

/** This type only exists for backwards compatibility, and should not be used in
new code. */
enum class SolverType {
  kClp,
  kCsdp,
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
