#pragma once

#include <cstdint>
#include <memory>

#include "drake/drakeOptimization_export.h"
#include "drake/solvers/solution_result.h"

namespace drake {
namespace solvers {
class OptimizationProblem;

/// Interface used by implementations of individual solvers.
class DRAKEOPTIMIZATION_EXPORT MathematicalProgramSolverInterface {
 public:
  virtual ~MathematicalProgramSolverInterface() = default;

  /// Returns true iff this solver was enabled at compile-time.
  virtual bool available() const = 0;

  /// Sets values for the decision variables on the given OptimizationProblem
  /// @p prog, or:
  ///  * If no solver is available, throws std::runtime_error
  ///  * If the solver returns an error, returns a nonzero SolutionResult.
  virtual SolutionResult Solve(OptimizationProblem& prog) const = 0;
};

/// A class for characterizing a mathematical program and choosing a solver.
/**
 * MathematicalProgram allows an OptimizationProblem to choose an appropriate
 * solver (MathematicalProgramSolverInterface) and to invoke that solver (via
 * the Solve() method).
 *
 * The problem simply calls the appropriate Add method of this class each time
 * it adds a cost or constraint.  Having done so, a call to Solve() will
 * invoke an appropriate solver, or else throw an exception if no solver is
 * available.
 */
class DRAKEOPTIMIZATION_EXPORT MathematicalProgram {
 public:
  MathematicalProgram();

  /* TODO(ggould-tri) The following methods might be desirable to fill out the
     optimization hierarchy:

     void AddIntegerVariable();
     void AddSumsOfSquaresConstraint();
     void AddLinearMatrixInequalityConstraint();
     void AddSecondOrderConeConstraint();
     void AddComplementarityConstraint();
  */

  /// Call if problem requires minimizing some f(x).
  void AddGenericCost();

  /// Call if problem requires bounding some f(x).
  void AddGenericConstraint();

  /// Call if problem requires minimizing some quadratic.
  void AddQuadraticCost();

  /// Call if problem requires bounding some quadratic.
  void AddQuadraticConstraint();

  /// Call if problem requires minimizing some linear expression.
  void AddLinearCost();

  /// Call if problem requires bounding some linear expression.
  void AddLinearConstraint();

  /// Call if problem requires exact satisfaction of linear equation.
  void AddLinearEqualityConstraint();

  /// Call if problem contains a linear complementarity optimization.
  void AddLinearComplementarityConstraint();

  /// Solve the given OptimizationProblem with an appropriate solver.
  /// Throws std::runtime_error if no appropriate solver is available.
  SolutionResult Solve(OptimizationProblem& prog) const;

 private:
  uint32_t required_capabilities_ {0};
  std::unique_ptr<MathematicalProgramSolverInterface> ipopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> nlopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> snopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> moby_lcp_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> linear_system_solver_;
  // TODO(ggould-tri) Add Gurobi here.
  // TODO(ggould-tri) Add Mosek here.
  // TODO(ggould-tri) Add SeDuMi here.
};

}  // namespace solvers
}  // namespace drake
