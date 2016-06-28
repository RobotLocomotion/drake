#include "MathematicalProgram.h"

#include "IpoptSolver.h"
#include "MobyLCP.h"
#include "NloptSolver.h"
#include "Optimization.h"
#include "SnoptSolver.h"

namespace drake {
namespace solvers {
MathematicalProgramInterface::~MathematicalProgramInterface() {}

namespace {

class MathematicalProgram : public MathematicalProgramInterface {
 public:
  virtual ~MathematicalProgram() {}

  /* these would be used to fill out the optimization hierarchy prototyped
     below
     virtual MathematicalProgramInterface* AddIntegerVariable() { return new
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
  MathematicalProgramInterface* AddQuadraticCost() override {
    return new MathematicalProgram;
  }
  MathematicalProgramInterface* AddLinearCost() override {
    return new MathematicalProgram;
  }
  MathematicalProgramInterface* AddGenericCost() override {
    return new MathematicalProgram;
  };
  MathematicalProgramInterface* AddGenericConstraint() override {
    return new MathematicalProgram;
  };
  MathematicalProgramInterface* AddLinearConstraint() override {
    return new MathematicalProgram;
  };
  MathematicalProgramInterface* AddLinearEqualityConstraint() override {
    return new MathematicalProgram;
  };
  MathematicalProgramInterface* AddLinearComplementarityConstraint() override {
    return new MathematicalProgram;
  };
  SolutionResult Solve(OptimizationProblem& prog) const override {
    throw std::runtime_error(
        "MathematicalProgram::Solve: "
        "No solver available for the given optimization problem!");
  }
};

class NonlinearProgram : public MathematicalProgram {
 public:
  // TODO(naveenoid) : bug exists in adding a Linear Quadratic or
  // Generic Cost to a NonlinearProgram
  MathematicalProgramInterface* AddGenericCost() override {
    return new NonlinearProgram;
  };
  MathematicalProgramInterface* AddGenericConstraint() override {
    return new NonlinearProgram;
  };
  MathematicalProgramInterface* AddLinearConstraint() override {
    return new NonlinearProgram;
  };
  MathematicalProgramInterface* AddLinearEqualityConstraint() override {
    return new NonlinearProgram;
  };
  MathematicalProgramInterface* AddLinearComplementarityConstraint() override {
    return new NonlinearProgram;
  }

  SolutionResult Solve(OptimizationProblem& prog) const override {
    if (snopt_solver.available()) {
      return snopt_solver.Solve(prog);
    }
    if (ipopt_solver.available()) {
      return ipopt_solver.Solve(prog);
    }
    if (nlopt_solver.available()) {
      return nlopt_solver.Solve(prog);
    }
    return MathematicalProgram::Solve(prog);
  }

 private:
  IpoptSolver ipopt_solver;
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
    virtual MathematicalProgramInterface* AddLinearInequalityConstraint() {
   return
    new LinearProgram; };
    };

    struct NonlinearComplementarityProblem : public NonlinearProgram {};
    struct LinearComplementarityProblem : public
    NonlinearComplementarityProblem {};
*/

class LinearComplementarityProblem : public MathematicalProgram {
 public:
  SolutionResult Solve(OptimizationProblem& prog) const override {
    // TODO(ggould-tri) given the Moby solver's meticulous use of temporaries,
    // it would be an easy performance win to reuse this solver object by making
    // a static place to store it.
    MobyLCPSolver solver;
    return solver.Solve(prog);
  }
  MathematicalProgramInterface* AddLinearComplementarityConstraint() override {
    return new LinearComplementarityProblem;
  };
  // TODO(naveenoid) : bug exists in adding a QuadraticCost to an LCP Problem.
  // Ideal fix involves rewrite of this entire class hierarchy.
};

class QuadraticProgram : public NonlinearProgram {
 public:
  MathematicalProgramInterface* AddQuadraticCost() override {
    return new QuadraticProgram;
  };
  MathematicalProgramInterface* AddLinearCost() override {
    return new QuadraticProgram;
  }

  MathematicalProgramInterface* AddLinearConstraint() override {
    return new QuadraticProgram;
  };
  MathematicalProgramInterface* AddLinearEqualityConstraint() override {
    return new QuadraticProgram;
  };

  SolutionResult Solve(OptimizationProblem& prog) const override {
    return NonlinearProgram::Solve(prog);
  }

  // TODO(naveenoid) : add Gurobi wrapper object, and solve here.
};

class LeastSquares : public QuadraticProgram {
 public:
  // LinearComplementarityProblem
  MathematicalProgramInterface* AddLinearEqualityConstraint() override {
    return new LeastSquares;
  };
  MathematicalProgramInterface* AddLinearComplementarityConstraint() override {
    return new LinearComplementarityProblem;
  };

  SolutionResult Solve(OptimizationProblem& prog) const override {
    size_t num_constraints = 0;
    for (auto const& binding : prog.linear_equality_constraints()) {
      num_constraints += binding.constraint()->A().rows();
    }

    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(
        num_constraints, prog.num_vars());
    // TODO(naveenoid) : use a sparse matrix here?
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
    return SolutionResult::kSolutionFound;
  }
};
}

std::shared_ptr<MathematicalProgramInterface>
MathematicalProgramInterface::GetLeastSquaresProgram() {
  return std::shared_ptr<MathematicalProgramInterface>(new LeastSquares);
}

MathematicalProgramSolverInterface::~MathematicalProgramSolverInterface() {}

}  // namespace solvers
}  // namespace drake
