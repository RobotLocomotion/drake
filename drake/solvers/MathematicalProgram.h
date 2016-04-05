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
class DRAKEOPTIMIZATION_EXPORT MathematicalProgramInterface {
 public:
  virtual ~MathematicalProgramInterface();

  /* these would be used to fill out the optimization hierarchy:

     virtual MathematicalProgramInterface* AddIntegerVariable() = 0;
     virtual MathematicalProgramInterface* AddLinearCost() = 0;
     virtual MathematicalProgramInterface* AddQuadraticCost() = 0;
     virtual MathematicalProgramInterface* AddCost() = 0;
     virtual MathematicalProgramInterface* AddSumsOfSquaresConstraint() = 0;
     virtual MathematicalProgramInterface* AddLinearMatrixInequalityConstraint()
     = 0;
     virtual MathematicalProgramInterface* AddSecondOrderConeConstraint() = 0;
     virtual MathematicalProgramInterface* AddComplementarityConstraint() = 0;
  */
  virtual MathematicalProgramInterface* AddGenericObjective() = 0;
  virtual MathematicalProgramInterface* AddGenericConstraint() = 0;
  virtual MathematicalProgramInterface* AddLinearConstraint() = 0;
  virtual MathematicalProgramInterface* AddLinearEqualityConstraint() = 0;
  virtual MathematicalProgramInterface*
  AddLinearComplementarityConstraint() = 0;

  virtual bool Solve(OptimizationProblem& prog) const = 0;

  static std::shared_ptr<MathematicalProgramInterface> GetLeastSquaresProgram();
};

/// Interface used by implementations of individual solvers.
class DRAKEOPTIMIZATION_EXPORT MathematicalProgramSolverInterface {
 public:
  virtual ~MathematicalProgramSolverInterface();
  virtual bool available() const = 0;
  virtual bool Solve(OptimizationProblem& prog) const = 0;
};
}

#endif  // DRAKE_SOLVERS_MATHEMATICALPROGRAM_H_
