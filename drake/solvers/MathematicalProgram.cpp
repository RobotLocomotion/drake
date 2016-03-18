#include "MathematicalProgram.h"
#include "Optimization.h"

namespace Drake {
MathematicalProgramInterface::~MathematicalProgramInterface() { }

namespace {

class MathematicalProgram : public MathematicalProgramInterface {
 public:
  virtual ~MathematicalProgram() { }

/* these would be used to fill out the optimization hierarchy prototyped
   below
   virtual MathematicalProgramInterface* addIntegerVariable() { return new
   MathematicalProgram; }

   virtual MathematicalProgramInterface* addLinearCost() { return new
   MathematicalProgram; }
   virtual MathematicalProgramInterface* addQuadraticCost() { return new
   MathematicalProgram; }
   virtual MathematicalProgramInterface* addCost() { return new 
   MathematicalProgram; }

   virtual MathematicalProgramInterface* addSumsOfSquaresConstraint() { return new
   MathematicalProgram; }
   virtual MathematicalProgramInterface* addLinearMatrixInequalityConstraint() {
   return new MathematicalProgram; }
   virtual MathematicalProgramInterface* addSecondOrderConeConstraint() { return new
   MathematicalProgram; }
   virtual MathematicalProgramInterface* addComplementarityConstraint() { return new
   MathematicalProgram; };
*/
  virtual MathematicalProgramInterface* addGenericObjective() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* addGenericConstraint() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* addLinearConstraint() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* addLinearEqualityConstraint() override {
    return new MathematicalProgram;
  };
  virtual bool solve(OptimizationProblem& prog) const override {
    throw std::runtime_error("not implemented yet");
  }
};

class NonlinearProgram : public MathematicalProgram {
 public:
    virtual MathematicalProgramInterface* addGenericObjective() override {
      return new NonlinearProgram;
    };
    virtual MathematicalProgramInterface* addGenericConstraint() override {
      return new NonlinearProgram;
    };
    virtual MathematicalProgramInterface* addLinearConstraint() override {
      return new NonlinearProgram;
    };
    virtual MathematicalProgramInterface* addLinearEqualityConstraint() override {
      return new NonlinearProgram;
    };

  virtual bool solve(OptimizationProblem& prog) const override {
    if (snopt_solver.available()) { return snopt_solver.solve(prog); }
    return MathematicalProgram::solve(prog);
  }

 private:
  MathematicalProgramSNOPTSolver snopt_solver;
};

/*  // Prototype of the more complete optimization problem class hiearchy (to
    be implemented only as needed)
    struct MixedIntegerNonlinearProgram : public MathematicalProgramInterface {};
    struct MixedIntegerSemidefiniteProgram : public
    MixedIntegerNonlinearProgram {};
    struct MixedIntegerSecondOrderConeProgram : public
    MixedIntegerSemidefiniteProgram {};
    struct MixedIntegerQuadraticProgram : public
    MixedIntegerSecondOrderConeProgram {};
    struct MixedIntegerLinearProgram : public MixedIntegerQuadraticProgram {};
    
    struct NonlinearProgram : public MixedIntegerNonlinearProgram {};
    struct SemidefiniteProgram : public NonlinearProgram, public
    MixedIntegerSemidefiniteProgram {};
    struct SecondOrderConeProgram : public SemidefiniteProgram, public
    MixedIntegerSecondOrderConeProgram {};
    struct QuadraticProgram : public SecondOrderConeProgram, public
    MixedIntegerQuadraticProgram {};
    struct LinearProgram : public QuadraticProgram, public
    MixedIntegerLinearProgram {
    virtual MathematicalProgramInterface* addLinearEqualityConstraint() { return new
    LinearProgram; };
    virtual MathematicalProgramInterface* addLinearInequalityConstraint() { return
    new LinearProgram; };
    };
    
    struct NonlinearComplementarityProblem : public NonlinearProgram {};
    struct LinearComplementarityProblem : public
    NonlinearComplementarityProblem {};
*/

class LeastSquares
    : public NonlinearProgram {  // public LinearProgram, public
 public:
  // LinearComplementarityProblem
  virtual MathematicalProgramInterface* addLinearEqualityConstraint() override {
    return new LeastSquares;
  };
  virtual bool solve(OptimizationProblem& prog) const override {
    size_t num_constraints = 0;
    for (auto const& binding : prog.getLinearEqualityConstraints()) {
      num_constraints += binding.getConstraint()->getMatrix().rows();
    }
    
    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(
        num_constraints, prog.getNumVars());  // todo: use a sparse matrix here?
    Eigen::VectorXd beq(num_constraints);
    
    size_t constraint_index = 0;
    for (auto const& binding :  prog.getLinearEqualityConstraints()) {
      auto const& c = binding.getConstraint();
      size_t n = c->getMatrix().rows();
      size_t var_index = 0;
      for (const DecisionVariableView& v : binding.getVariableList()) {
        Aeq.block(constraint_index, v.index(), n, v.size()) =
            c->getMatrix().middleCols(var_index, v.size());
        var_index += v.size();
      }
      beq.segment(constraint_index, n) =
          c->getLowerBound();  // = c->getUpperBound() since it's an equality
      // constraint
      constraint_index += n;
    }
    
    std::cout << "LeastSquares::solve():\n"
          << "  - aeq =\n" << Aeq << "\n"
          << "  - beq =\n" << beq << "\n"
          << "  - Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).singularValues()=\n"
          << Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).singularValues()
          << std::endl;

    // least-squares solution
    prog.setDecisionVariableValues(
        Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beq));
    return true;
  }
};
}

std::shared_ptr<MathematicalProgramInterface> 
MathematicalProgramInterface::getLeastSquaresProgram() {
  return std::shared_ptr<MathematicalProgramInterface>(new LeastSquares);
}

MathematicalProgramSolverInterface::~MathematicalProgramSolverInterface() { }

}
