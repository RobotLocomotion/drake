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
  , kDReal [[deprecated("DRAKE DEPRECATED: dReal support is being withdrawn from Drake; for details, see https://github.com/RobotLocomotion/drake/pull/18156. The deprecated code will be removed from Drake on or after 2023-02-01")]]  // NOLINT
  , kIbex  [[deprecated("DRAKE DEPRECATED: IBEX support is being withdrawn from Drake; for details, see https://github.com/RobotLocomotion/drake/pull/18156. The deprecated code will be removed from Drake on or after 2023-02-01")]]  // NOLINT
};

}  // namespace solvers
}  // namespace drake
