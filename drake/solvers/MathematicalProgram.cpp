#include "MathematicalProgram.h"

#include "IpoptSolver.h"
#include "MobyLCP.h"
#include "NloptSolver.h"
#include "Optimization.h"
#include "SnoptSolver.h"

namespace drake {
namespace solvers {

namespace {

enum ProblemAttributes {
  kNoCapabilities = 0,
  kError = 1 << 0,  ///< Do not use, to avoid & vs. && typos.
  kGenericCost                     = 1 << 1,
  kGenericConstraint               = 1 << 2,
  kQuadraticCost                   = 1 << 3,
  kQuadraticConstraint             = 1 << 4,
  kLinearCost                      = 1 << 5,
  kLinearConstraint                = 1 << 6,
  kLinearEqualityConstraint        = 1 << 7,
  kLinearComplementarityConstraint = 1 << 8
};
typedef uint32_t AttributesSet;

// TODO(ggould-tri) Refactor these capability advertisements into the
// solver wrappers themselves.

// Solver for simple linear systems of equalities
AttributesSet kLeastSquaresCapabilities = kLinearEqualityConstraint;

// Solver for Linear Complementarity Problems (LCPs)
AttributesSet kMobyLcpCapabilities = kLinearComplementarityConstraint;

// Solver for Quadratic Programs (QPs); commented out until Gurobi is ready
// to land.
// AttributesSet kGurobiCapabilities = (
//     kLinearEqualityConstraint | kLinearInequalityConstraint |
//     kLinearCost | kQuadraticCost);

// Solvers for generic systems of constraints and costs.
AttributesSet kGenericSolverCapabilities = (
    kGenericCost | kGenericConstraint |
    kQuadraticCost | kQuadraticConstraint |
    kLinearCost | kLinearConstraint | kLinearEqualityConstraint);

// Returns true iff no capabilities are in required and not in available.
bool is_satisfied(AttributesSet required, AttributesSet available) {
  return ((required & ~available) == kNoCapabilities);
}


class DRAKEOPTIMIZATION_EXPORT LeastSquaresSolver :
      public MathematicalProgramSolverInterface {
  ~LeastSquaresSolver() override {};

  bool available() const override { return true; }

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

}  // anon namespace


MathematicalProgram::MathematicalProgram()
    : required_capabilities_(kNoCapabilities),
      ipopt_solver_(new IpoptSolver()),
      nlopt_solver_(new NloptSolver()),
      snopt_solver_(new SnoptSolver()),
      moby_lcp_solver_(new MobyLCPSolver()),
      least_squares_solver_(new LeastSquaresSolver()) {}

void MathematicalProgram::AddGenericCost() {
  required_capabilities_ |= kGenericCost;
}
void MathematicalProgram::AddGenericConstraint() {
  required_capabilities_ |= kGenericConstraint;
}
void MathematicalProgram::AddQuadraticCost() {
  required_capabilities_ |= kQuadraticCost;
}
void MathematicalProgram::AddQuadraticConstraint() {
  required_capabilities_ |= kQuadraticConstraint;
}
void MathematicalProgram::AddLinearCost() {
  required_capabilities_ |= kLinearCost;
}
void MathematicalProgram::AddLinearConstraint() {
  required_capabilities_ |= kLinearConstraint;
}
void MathematicalProgram::AddLinearEqualityConstraint() {
  required_capabilities_ |= kLinearEqualityConstraint;
}
void MathematicalProgram::AddLinearComplementarityConstraint() {
  required_capabilities_ |= kLinearComplementarityConstraint;
}

SolutionResult MathematicalProgram::Solve(OptimizationProblem& prog) const {
  // This implementation is simply copypasta for now; in the future we will
  // want to tweak the order of preference of solvers based on the types of
  // constraints present.
  if (is_satisfied(required_capabilities_, kLeastSquaresCapabilities) &&
      least_squares_solver_->available()) {
    // TODO(ggould-tri) Also allow quadratic objectives whose matrix is
    // Identity: This is the objective function the solver uses anyway when
    // underconstrainted, and is fairly common in real-world problems.
    return least_squares_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_, kMobyLcpCapabilities) &&
             moby_lcp_solver_->available()) {
    return moby_lcp_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             snopt_solver_->available()) {
    return snopt_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             ipopt_solver_->available()) {
    return ipopt_solver_->Solve(prog);
  } else if (is_satisfied(required_capabilities_, kGenericSolverCapabilities) &&
             nlopt_solver_->available()) {
    return nlopt_solver_->Solve(prog);
  } else {
    throw std::runtime_error(
        "MathematicalProgram::Solve: "
        "No solver available for the given optimization problem!");
  }
}

}  // namespace solvers
}  // namespace drake
