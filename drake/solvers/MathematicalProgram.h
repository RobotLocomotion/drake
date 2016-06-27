#pragma once

#include <memory>

#include "drake/drakeOptimization_export.h"
#include "drake/solvers/solution_result.h"

namespace drake {
namespace solvers {
class OptimizationProblem;

/// Interface used by implementations of individual solvers.
class DRAKEOPTIMIZATION_EXPORT MathematicalProgramSolverInterface {
 public:
  virtual ~MathematicalProgramSolverInterface() {};
  virtual bool available() const = 0;
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

  /* these would be used to fill out the optimization hierarchy:

     virtual MathematicalProgramInterface* AddIntegerVariable() = 0;
     virtual MathematicalProgramInterface* AddSumsOfSquaresConstraint() = 0;
     virtual MathematicalProgramInterface* AddLinearMatrixInequalityConstraint()
     = 0;
     virtual MathematicalProgramInterface* AddSecondOrderConeConstraint() = 0;
     virtual MathematicalProgramInterface* AddComplementarityConstraint() = 0;
  */
  void AddGenericObjective();
  void AddGenericConstraint();
  void AddLinearObjective();
  void AddLinearConstraint();
  void AddQuadraticObjective();
  void AddQuadraticConstraint();
  void AddLinearEqualityConstraint();
  void AddLinearComplementarityConstraint();

  SolutionResult Solve(OptimizationProblem& prog) const;

 private:
  int required_capabilities_;
  std::unique_ptr<MathematicalProgramSolverInterface> ipopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> nlopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> snopt_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> moby_lcp_solver_;
  std::unique_ptr<MathematicalProgramSolverInterface> least_squares_solver_;
};

}  // namespace solvers
}  // namespace drake
