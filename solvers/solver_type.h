#pragma once

namespace drake {
namespace solvers {

/** This type only exists for backwards compatiblity, and should not be used in
new code. */
enum class SolverType {
  kClp,
  kCsdp,
  kDReal,
  kEqualityConstrainedQP,
  kGurobi,
  kIbex,
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
