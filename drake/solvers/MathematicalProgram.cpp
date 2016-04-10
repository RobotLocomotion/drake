#include "MathematicalProgram.h"

#include "MobyLCP.h"
#include "NloptSolver.h"
#include "Optimization.h"
#include "SnoptSolver.h"

namespace Drake {
MathematicalProgramInterface::~MathematicalProgramInterface() {}

namespace {

class MathematicalProgram : public MathematicalProgramInterface {
 public:
  virtual ~MathematicalProgram() {}

  /* these would be used to fill out the optimization hierarchy prototyped
     below
     virtual MathematicalProgramInterface* AddIntegerVariable() { return new
     MathematicalProgram; }

     virtual MathematicalProgramInterface* AddLinearCost() { return new
     MathematicalProgram; }
     virtual MathematicalProgramInterface* AddQuadraticCost() { return new
     MathematicalProgram; }
     virtual MathematicalProgramInterface* AddCost() { return new
     MathematicalProgram; }

     virtual MathematicalProgramInterface* AddSumsOfSquaresConstraint() { return
     new
     MathematicalProgram; }
     virtual MathematicalProgramInterface* AddLinearMatrixInequalityConstraint()
     {
     return new MathematicalProgram; }
     virtual MathematicalProgramInterface* AddSecondOrderConeConstraint() {
     return new
     MathematicalProgram; }
     virtual MathematicalProgramInterface* AddComplementarityConstraint() {
     return new
     MathematicalProgram; };
  */
  virtual MathematicalProgramInterface* AddGenericObjective() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* AddGenericConstraint() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* AddLinearConstraint() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* AddLinearEqualityConstraint() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* AddLinearComplementarityConstraint()
      override {
    return new MathematicalProgram;
  };
  virtual bool Solve(OptimizationProblem& prog) const override {
    throw std::runtime_error(
        "MathematicalProgram::Solve: "
        "No solver available for the given optimization problem!");
  }
};

class NonlinearProgram : public MathematicalProgram {
 public:
  virtual MathematicalProgramInterface* AddGenericObjective() override {
    return new NonlinearProgram;
  };
  virtual MathematicalProgramInterface* AddGenericConstraint() override {
    return new NonlinearProgram;
  };
  virtual MathematicalProgramInterface* AddLinearConstraint() override {
    return new NonlinearProgram;
  };
  virtual MathematicalProgramInterface* AddLinearEqualityConstraint() override {
    return new NonlinearProgram;
  };
  virtual MathematicalProgramInterface* AddLinearComplementarityConstraint()
      override {
    return new NonlinearProgram;
  }

  virtual bool Solve(OptimizationProblem& prog) const override {
    if (snopt_solver.available()) {
      return snopt_solver.Solve(prog);
    }
    if (nlopt_solver.available()) {
      return nlopt_solver.Solve(prog);
    }
    return MathematicalProgram::Solve(prog);
  }

 private:
  NloptSolver nlopt_solver;
  SnoptSolver snopt_solver;
};

/*  // Prototype of the more complete optimization problem class hiearchy (to
    be implemented only as needed)
    struct MixedIntegerNonlinearProgram : public MathematicalProgramInterface
   {};
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
    virtual MathematicalProgramInterface* AddLinearEqualityConstraint() { return
   new
    LinearProgram; };
    virtual MathematicalProgramInterface* AddLinearInequalityConstraint() { return
    new LinearProgram; };
    };

    struct NonlinearComplementarityProblem : public NonlinearProgram {};
    struct LinearComplementarityProblem : public
    NonlinearComplementarityProblem {};
*/

class LinearComplementarityProblem : public MathematicalProgram {
 public:
  virtual bool Solve(OptimizationProblem& prog) const override {
    // TODO ggould given the Moby solver's meticulous use of temporaries, it
    // would be an easy performance win to reuse this solver object by making
    // a static place to store it.
    MobyLCPSolver solver;
    return solver.Solve(prog);
  }
  virtual MathematicalProgramInterface* AddLinearComplementarityConstraint()
      override {
    return new LinearComplementarityProblem;
  };
};

class LeastSquares : public NonlinearProgram {  // public LinearProgram, public
 public:
  // LinearComplementarityProblem
  virtual MathematicalProgramInterface* AddLinearEqualityConstraint() override {
    return new LeastSquares;
  };
  virtual MathematicalProgramInterface* AddLinearComplementarityConstraint()
      override {
    return new LinearComplementarityProblem;
  };

  virtual bool Solve(OptimizationProblem& prog) const override {
    size_t num_constraints = 0;
    for (auto const& binding : prog.linear_equality_constraints()) {
      num_constraints += binding.constraint()->A().rows();
    }

    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(
        num_constraints, prog.num_vars());  // todo: use a sparse matrix here?
    Eigen::VectorXd beq(num_constraints);

    size_t constraint_index = 0;
    for (auto const& binding : prog.linear_equality_constraints()) {
      auto const& c = binding.constraint();
      size_t n = c->A().rows();
      size_t var_index = 0;
      for (const DecisionVariableView& v : binding.variable_list()) {
        Aeq.block(constraint_index, v.index(), n, v.size()) =
            c->A().middleCols(var_index, v.size());
        var_index += v.size();
      }
      beq.segment(constraint_index, n) =
          c->lower_bound();  // = c->upper_bound() since it's an equality
      // constraint
      constraint_index += n;
    }

    // least-squares solution
    prog.SetDecisionVariableValues(
        Aeq.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(beq));
    return true;
  }
};
}

std::shared_ptr<MathematicalProgramInterface>
MathematicalProgramInterface::GetLeastSquaresProgram() {
  return std::shared_ptr<MathematicalProgramInterface>(new LeastSquares);
}

MathematicalProgramSolverInterface::~MathematicalProgramSolverInterface() {}
}
