#ifndef DRAKE_SOLVERS_MATHEMATICALPROGRAM_H_
#define DRAKE_SOLVERS_MATHEMATICALPROGRAM_H_

#include <memory>

#include "drake/drakeOptimization_export.h"

namespace Drake {
class OptimizationProblem;

// uses virtual methods to crawl up the complexity hiearchy as new decision
// variables and constraints are added to the program
// note that there is dynamic allocation happening in here, but on a structure
// of negligible size.  (is there a better way?)
class DRAKEOPTIMIZATION_EXPORT  MathematicalProgramInterface {
 public:
  virtual ~MathematicalProgramInterface();

  /* these would be used to fill out the optimization hierarchy:

     virtual MathematicalProgramInterface* addIntegerVariable() = 0;
     virtual MathematicalProgramInterface* addLinearCost() = 0;
     virtual MathematicalProgramInterface* addQuadraticCost() = 0;
     virtual MathematicalProgramInterface* addCost() = 0;
     virtual MathematicalProgramInterface* addSumsOfSquaresConstraint() = 0;
     virtual MathematicalProgramInterface* addLinearMatrixInequalityConstraint() = 0;
     virtual MathematicalProgramInterface* addSecondOrderConeConstraint() = 0;
     virtual MathematicalProgramInterface* addComplementarityConstraint() = 0;
  */
  virtual MathematicalProgramInterface* add_generic_objective() = 0;
  virtual MathematicalProgramInterface* add_generic_constraint() = 0;
  virtual MathematicalProgramInterface* add_linear_constraint() = 0;
  virtual MathematicalProgramInterface* add_linear_equality_constraint() = 0;
  virtual MathematicalProgramInterface*
      add_linear_complementarity_constraint() = 0;

  virtual bool Solve(OptimizationProblem& prog) const = 0;

  static std::shared_ptr<MathematicalProgramInterface>
      get_least_squares_program();
};

/// Interface used by implementations of individual solvers.
class DRAKEOPTIMIZATION_EXPORT MathematicalProgramSolverInterface {
 public:
  virtual ~MathematicalProgramSolverInterface();
  virtual bool available() const = 0;
  virtual bool Solve(OptimizationProblem& prog) const = 0;
};

class DRAKEOPTIMIZATION_EXPORT MathematicalProgramSNOPTSolver :
      public MathematicalProgramSolverInterface  {
 public:
  // This solver is implemented in various pieces depending on if
  // SNOPT was available during compilation.
  virtual bool available() const override;
  virtual bool Solve(OptimizationProblem& prog) const override;
};
}

#endif  // DRAKE_SOLVERS_MATHEMATICALPROGRAM_H_
