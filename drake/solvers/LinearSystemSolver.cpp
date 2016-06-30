#include "drake/solvers/LinearSystemSolver.h"

#include <cstring>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/core/Gradient.h"
#include "drake/solvers/Optimization.h"

using Drake::TaylorVecXd;

namespace drake {
namespace solvers {


/*
TODO: 
      separate into an object file for linear system and rename appropriately
      flesh out unconstrainted / equality-constrained QP?
*/
      
bool LinearSystemSolver::available() const { 
  return true; 
}

SolutionResult LinearSystemSolver::Solve(OptimizationProblem& prog) const {
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

  prog.SetSolverResult("Linear System Solver", 0);
  return SolutionResult::kSolutionFound;
}

}  // namespace drake
}  // namespace solvers
