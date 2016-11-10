#include "drake/solvers/equality_constrained_qp_solver.h"

#include <cstring>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

bool EqualityConstrainedQPSolver::available() const { return true; }

/**
 * Determines non-redundant set of linear equality constraints using Cholesky 
 * factorization and rank-1 updates. The idea is to determine whether the 
 * linear equality constraints have full row rank by determining whether `A*A'`
 * is positive definite. Constraints that make A nearly non-full rank should
 * be eliminated through the somewhat touchy Cholesky factorization.
 */
void EqualityConstrainedQPSolver::ComputeNonRedundantEqualityConstraints(
    MathematicalProgram& prog) const){
  // Get the number of program variables.

  // Setup the indices to collect.
  std::vector<int> to_use;
  
  // Find the first non-zero row of A
  int row_index = -1;
  for (auto const& binding : prog.linear_equality_constraints())
    for (int i=0; i < binding.constraint()->A().rows(); ++i)
      if (binding.constraint()->A().row(i).normInf() > kZeroTol) {
        row_index = i;
        break;
      }

  // If the row index is not set, remove all A/b constraints.

  // Store the reference to the row.

  // setup AA' and compute the Cholesky factorization
  MatrixXd AAT(1,1) = binding.constraint()->A().row(i).transpose() *
                binding.constraint()->A().row(i);

  
  // Compute the Cholesky factorization of this row of A.
  LLT<MatrixXd> llt(AAT);
  DRAKE_DEMAND(llt.info() == Eigen::Success);

  // Loop until have gone all of the way through A.
  
    // Attempt to compute a rank-1 update to the Cholesky factorization.
  
    // Attempt was not successful, revert the Cholesky factorization.
  
    // Attempt was successful; set the corresponding index in to_use to "true"
  
  // Get all selected rows of A/b.
}

SolutionResult EqualityConstrainedQPSolver::Solve(
    MathematicalProgram& prog) const {
  // Given a QP with equality constraints, we can use the KKT conditions
  // for the optimal solution to write a linear system that produces
  // our answer.
  // (see https://www.math.uh.edu/~rohop/fall_06/Chapter3.pdf)
  // Does not explicitly handle undertermined cases.

  // TODO(gizatt@mit.edu): This way of detecting whether our
  // constraints are satisfied will scale poorly as we add
  // more constraint and cost types, and more solvers, as
  // a change to any of these accessors or addition of new
  // cost or constraint types will require editing this check
  // in every solver.
  DRAKE_ASSERT(prog.generic_constraints().empty());
  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.linear_constraints().empty());
  DRAKE_ASSERT(prog.bounding_box_constraints().empty());
  DRAKE_ASSERT(prog.linear_complementarity_constraints().empty());

  size_t num_constraints = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    num_constraints += binding.constraint()->A().rows();
  }

  // There are three ways to solve the KKT subproblem for convex QPs.
  // Formally, we want to solve:
  // | Q  -A' | | x | = | -c  |
  // | A   0  | | y | = |  b |
  // for problem variables x and Lagrange multiplier variables y. 
  // Approach 1: compute a LDL' factorization
  // Approach 2: use the Schur complement ("range space" approach)
  // Approach 3: use the nullspace of A ("null space" approach)
  // All of these approaches assume that A has full row rank (i.e., there are
  // no redundant constraints).

  // Determine non-redundant set of constraints
  
  // Check for positive definite Hessian matrix.
  
    // Matrix is positive definite. Solve A*inv(Q)*A'y = A*inv(Q)*c - b for `y`.

    // Solve Q*x = A'y - c 
  
  // The following code assumes that the Hessian is not positive definite.
  // It uses the LDL' factorization.

  // The expanded problem introduces a Lagrangian multiplier for each
  // linear equality constraint.
  size_t num_full_vars = prog.num_vars() + num_constraints;
  Eigen::MatrixXd A_full = Eigen::MatrixXd::Zero(num_full_vars, num_full_vars);
  Eigen::VectorXd b_full = Eigen::VectorXd::Zero(num_full_vars);

  // Assemble the A and b matrices -- first by summing over
  // quadratic costs ...
  for (auto const& binding : prog.quadratic_costs()) {
    size_t index = 0;
    for (const DecisionVariableView& v : binding.variable_list()) {
      A_full.block(v.index(), v.index(), v.size(), v.size()) +=
          binding.constraint()->Q().block(index, index, v.size(), v.size());
      b_full.segment(v.index(), v.size()) -=
          binding.constraint()->b().segment(index, v.size());
      index += v.size();
    }
  }

  // ... and then the Lagrangian multiplier penalty entries:
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
        c->lower_bound().segment(0, n);  // = c->upper_bound() since it's
                                         //  an equality constraint
    constraint_index += n;
  }

  // Compute the least-squares solution.
  Eigen::VectorXd sol =
      A_full.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_full);
  prog.SetDecisionVariableValues(sol.segment(0, prog.num_vars()));

  prog.SetSolverResult("Equality Constrained QP Solver", 0);
  return SolutionResult::kSolutionFound;
}

}  // namespace solvers
}  // namespace drake
