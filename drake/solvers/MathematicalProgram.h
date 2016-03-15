#ifndef DRAKE_MATHEMATICAL_PROGRAM_H
#define DRAKE_MATHEMATICAL_PROGRAM_H

#include <memory>

namespace Drake {
class OptimizationProblem;

// uses virtual methods to crawl up the complexity hiearchy as new decision
// variables and constraints are added to the program
// note that there is dynamic allocation happening in here, but on a structure
// of negligible size.  (is there a better way?)
class MathematicalProgramInterface {
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
  virtual MathematicalProgramInterface* addGenericObjective() = 0;
  virtual MathematicalProgramInterface* addGenericConstraint() = 0;
  virtual MathematicalProgramInterface* addLinearConstraint() = 0;
  virtual MathematicalProgramInterface* addLinearEqualityConstraint() = 0;
  virtual bool solve(OptimizationProblem& prog) const = 0;

  static std::shared_ptr<MathematicalProgramInterface> getLeastSquaresProgram();
};

/// Interface used by implementations of individual solvers.
class MathematicalProgramSolverInterface {
 public:
  virtual ~MathematicalProgramSolverInterface();
  virtual bool available() const = 0;
  virtual bool solve(OptimizationProblem& prog) const = 0;
};

class MathematicalProgramSNOPTSolver : 
      public MathematicalProgramSolverInterface  {
 public:
  // This solver is implemented in various pieces depending on if
  // SNOPT was available during compilation.
  virtual bool available() const override;
  virtual bool solve(OptimizationProblem& prog) const override;
};
}

#endif
