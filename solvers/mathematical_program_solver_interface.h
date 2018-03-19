#pragma once

#include <ostream>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/solvers/solver_id.h"

namespace drake {
namespace solvers {
class MathematicalProgram;

enum SolutionResult {
  kSolutionFound = 0,           ///< Found the optimal solution.
  kInvalidInput = -1,           ///< Invalid input.
  kInfeasibleConstraints = -2,  ///< The primal is infeasible.
  kUnbounded = -3,              ///< The primal is unbounded.
  kUnknownError = -4,           ///< Unknown error.
  kInfeasible_Or_Unbounded =
      -5,                ///< The primal is either infeasible or unbounded.
  kIterationLimit = -6,  ///< Reaches the iteration limits.
  kDualInfeasible = -7,  ///< Dual problem is infeasible. In this case we cannot
                         /// infer the status of the primal problem.
};

/**
 * The class implementations of the class use to report their results to the
 * mathematical program. It is guaranteed to have a defined solver id; all other
 * fields can be left undefined. Reading those values should be guarded by a
 * test on whether the field has been defined.
 */
class SolverResult {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverResult)

  explicit SolverResult(const SolverId& solver_id) : solver_id_(solver_id) {}

  const SolverId& solver_id() const { return solver_id_; }

  void set_decision_variable_values(
      const Eigen::Ref<const Eigen::VectorXd>& values) {
    decision_variable_values_.reset();
    decision_variable_values_.emplace(values);
  }

  const optional<Eigen::VectorXd>& decision_variable_values() const {
    return decision_variable_values_;
  }

  void set_optimal_cost(double optimal_cost) { optimal_cost_ = optimal_cost; }

  const optional<double>& optimal_cost() const { return optimal_cost_; }

  void set_optimal_cost_lower_bound(double val) {
    optimal_cost_lower_bound_ = val;
  }

  const optional<double>& optimal_cost_lower_bound() const {
    return optimal_cost_lower_bound_;
  }

 private:
  SolverId solver_id_;
  optional<Eigen::VectorXd> decision_variable_values_{nullopt};
  optional<double> optimal_cost_{nullopt};
  optional<double> optimal_cost_lower_bound_{nullopt};
};

/// Interface used by implementations of individual solvers.
class MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MathematicalProgramSolverInterface)

  MathematicalProgramSolverInterface() = default;
  virtual ~MathematicalProgramSolverInterface() = default;

  /// Returns true iff this solver was enabled at compile-time.
  virtual bool available() const = 0;

  /// Sets values for the decision variables on the given MathematicalProgram
  /// @p prog, or:
  ///  * If no solver is available, throws std::runtime_error
  ///  * If the solver returns an error, returns a nonzero SolutionResult.
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  virtual SolutionResult Solve(MathematicalProgram& prog) const = 0;

  /// Returns the identifier of this solver.
  virtual SolverId solver_id() const = 0;
};

}  // namespace solvers
}  // namespace drake
