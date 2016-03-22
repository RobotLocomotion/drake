#include "MathematicalProgram.h"

#include "MobyLCP.h"
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
  virtual MathematicalProgramInterface* add_generic_objective() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* add_generic_constraint() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* add_linear_constraint() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface* add_linear_equality_constraint() override {
    return new MathematicalProgram;
  };
  virtual MathematicalProgramInterface*
  add_linear_complementarity_constraint() override {
    return new MathematicalProgram;
  };
  virtual bool Solve(OptimizationProblem& prog) const override {
    throw std::runtime_error("not implemented yet");
  }
};

class NonlinearProgram : public MathematicalProgram {
 public:

 virtual MathematicalProgramInterface* add_generic_objective() override {
    return new NonlinearProgram;
  };
  virtual MathematicalProgramInterface* add_generic_constraint() override {
    return new NonlinearProgram;
  };
  virtual MathematicalProgramInterface* add_linear_constraint() override {
    return new NonlinearProgram;
  };
  virtual MathematicalProgramInterface* add_linear_equality_constraint() override {
    return new NonlinearProgram;
  };
  virtual MathematicalProgramInterface*
  add_linear_complementarity_constraint() override {
    return new NonlinearProgram;
  }

  virtual bool Solve(OptimizationProblem& prog) const override {
    if (snopt_solver.available()) { return snopt_solver.Solve(prog); }
    try {
      return MathematicalProgram::Solve(prog);
    } catch (const std::runtime_error& e) {
      throw std::runtime_error("SNOPT unavailable");
    }
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
    virtual MathematicalProgramInterface* add_linear_equality_constraint() { return new
    LinearProgram; };
    virtual MathematicalProgramInterface* add_linear_inequality_constraint() { return
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
  virtual MathematicalProgramInterface*
  add_linear_complementarity_constraint() override {
    return new LinearComplementarityProblem;
  };
};

class LeastSquares
    : public NonlinearProgram {  // public LinearProgram, public
 public:
  // LinearComplementarityProblem
  virtual MathematicalProgramInterface* add_linear_equality_constraint() override {
    return new LeastSquares;
  };
  virtual MathematicalProgramInterface*
  add_linear_complementarity_constraint() override {
    return new LinearComplementarityProblem;
  };

  virtual bool Solve(OptimizationProblem& prog) const override {
    size_t num_constraints = 0;
    for (auto const& binding : prog.get_linear_equality_constraints()) {
      num_constraints += binding.get_constraint()->get_matrix().rows();
    }

    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(
        num_constraints, prog.get_num_vars());
    Eigen::VectorXd beq(num_constraints);

    size_t constraint_index = 0;
    for (auto const& binding :  prog.get_linear_equality_constraints()) {
      auto const& c = binding.get_constraint();
      size_t n = c->get_matrix().rows();
      size_t var_index = 0;
      for (const DecisionVariableView& v : binding.get_variable_list()) {
        Aeq.block(constraint_index, v.index(), n, v.size()) =
            c->get_matrix().middleCols(var_index, v.size());
        var_index += v.size();
      }
      beq.segment(constraint_index, n) =
          c->get_lower_bound();  // = c->get_upper_bound() since it's an
                                 // equality constraint
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
MathematicalProgramInterface::get_least_squares_program() {
  return std::shared_ptr<MathematicalProgramInterface>(new LeastSquares);
}

MathematicalProgramSolverInterface::~MathematicalProgramSolverInterface() { }

}
