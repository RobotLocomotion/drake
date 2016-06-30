#include "drake/solvers/EqualityConstrainedQPSolver.h"

#include <cstring>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/core/Gradient.h"
#include "drake/solvers/Optimization.h"

using Drake::TaylorVecXd;

namespace drake {
namespace solvers {
      
bool EqualityConstrainedQPSolver::available() const { 
  return true; 
}

SolutionResult EqualityConstrainedQPSolver::Solve(OptimizationProblem& prog) const {
  // Given a QP with equality constraints, we can use the KKT conditions
  // for the optimal solution to write a linear system that produces
  // our answer.
  // (see https://www.math.uh.edu/~rohop/fall_06/Chapter3.pdf)

  // todo: this needs to be tested with sums of quadratic costs
  // going in, as well as with quadratic costs that cover different
  // views and don't cover the complete Q matrix together (to see
  // how this responds to underdetermined situations)
  
  size_t num_constraints = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    num_constraints += binding.constraint()->A().rows();
  }

  // expanded problem introduces a lagragian multiplier for each
  // linear equality constraint
  size_t num_full_vars = prog.num_vars() + num_constraints;
  Eigen::MatrixXd A_full = Eigen::MatrixXd::Zero( num_full_vars,
    num_full_vars);
  Eigen::VectorXd b_full(num_full_vars);

  // assemble the A and b matrices -- first by summing over 
  //   quadratic costs
  for (auto const& binding : prog.quadratic_costs()) {
    for (const DecisionVariableView& v : binding.variable_list()) {
      A_full.block(v.index(), v.index(), v.size(), v.size()) += 
        binding.constraint()->Q().block(v.index(), v.index(), 
                                        v.size(), v.size());
      b_full.segment(v.index(), v.size()) += 
        - binding.constraint()->b().segment(v.index(), v.size());
    }
  }

  // then the lagrangian multiplier penalty entries
  size_t constraint_index = prog.num_vars();
  for (auto const& binding : prog.linear_equality_constraints()) {
    auto const& c = binding.constraint();
    size_t n = c->A().rows();
    size_t var_index = 0;
    for (const DecisionVariableView& v : binding.variable_list()) {
      A_full.block(constraint_index, v.index(), n, v.size()) =
          c->A().middleCols(var_index, v.size());
      A_full.block(v.index(), constraint_index, v.size(), n) =
          (c->A().middleCols(var_index, v.size())).transpose();
      var_index += v.size();
    }
    b_full.segment(constraint_index, n) =
        c->lower_bound();  // = c->upper_bound() since it's an equality
    // constraint
    constraint_index += n;
  }

  // least-squares solution
  Eigen::VectorXd sol = A_full.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_full);
  prog.SetDecisionVariableValues(sol.segment(0, prog.num_vars()));

  prog.SetSolverResult("Equality Constrained QP Solver", 0);
  return SolutionResult::kSolutionFound;
}

}  // namespace drake
}  // namespace solvers
