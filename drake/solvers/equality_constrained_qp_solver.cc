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

SolutionResult EqualityConstrainedQPSolver::Solve(
    MathematicalProgram& prog) const {
  // There are three ways to solve the KKT subproblem for convex QPs.
  // Formally, we want to solve:
  // | G  A' | | x | = | -c |
  // | A  0  | | y | = |  b |
  // for problem variables x and Lagrange multiplier variables y. This
  // corresponds to the QP:
  // minimize 1/2 x'*G*x + c'*x
  // s.t.:    A*x = b
  // Approach 1: Solve the full linear system above.
  // Approach 2: Use the Schur complement ("range space" approach).
  // Approach 3: Use the nullspace of A ("null space" approach).

  // The QP approach attempts Approach (2) and falls back to Approach (1).
  // The Approach (1) implementation is vestigial and uses a singular value
  // decomposition. Approach (1) could be made slightly faster by using a
  // QR factorization instead. It could be made considerably faster than that
  // if the A matrix were known to have full row rank, which would allow
  // a symmetric LDL' factorization to be used. As long as the quadratic
  // cost matrix is symmetric and positive definite, both approaches should
  // yield the same optimal point; the same set of Lagrange multipliers is
  // not guaranteed (but the Lagrange multipliers are not currently being
  // returned to the user).
  //
  // This implementation was conducted using [Nocedal 1999], Ch. 16 (Quadratic
  // Programming).  It is recommended that programmers desiring to modify this
  // code have a solid understanding of equality constrained quadratic
  // programming before proceeding.
  // - J. Nocedal and S. Wright. Numerical Optimization. Springer, 1999.
  DRAKE_ASSERT(prog.generic_constraints().empty());
  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.linear_constraints().empty());
  DRAKE_ASSERT(prog.bounding_box_constraints().empty());
  DRAKE_ASSERT(prog.linear_complementarity_constraints().empty());

  size_t num_constraints = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    num_constraints += binding.constraint()->A().rows();
  }

  // Setup the quadratic cost matrix and linear cost vector.
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(prog.num_vars(), prog.num_vars());
  Eigen::VectorXd c = Eigen::VectorXd::Zero(prog.num_vars());
  for (auto const& binding : prog.quadratic_costs()) {
    size_t index = 0;
    const auto& Q = binding.constraint()->Q();
    const auto& b = binding.constraint()->b();
    for (const auto& v : binding.variable_list().variables()) {
      int num_v_variables = v.rows();
      DRAKE_ASSERT(v.cols() == 1);
      for (int i = 0; i < num_v_variables; ++i) {
        for (int j = 0; j < num_v_variables; ++j) {
          G(v(i, 0).index(), v(j, 0).index()) += Q(index + i, index + j);
        }
        c(v(i, 0).index()) += b(index + i);
      }
      index += num_v_variables;
    }
  }

  // Setup the linear constraints.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, prog.num_vars());
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints);
  int constraint_index = 0;
  for (auto const& binding : prog.linear_equality_constraints()) {
    auto const& bc = binding.constraint();
    size_t n = bc->A().rows();
    size_t var_index = 0;
    for (const auto& v : binding.variable_list().variables()) {
      DRAKE_ASSERT(v.cols() == 1);
      int num_v_variables = v.rows();
      for (int i = 0; i < num_v_variables; ++i) {
        A.block(constraint_index, v(i, 0).index(), n, 1) =
            bc->A().col(var_index + i);
      }
      var_index += num_v_variables;
    }
    b.segment(constraint_index, n) =
        bc->lower_bound().segment(0, n);  // = c->upper_bound() since it's
    //  an equality constraint
    constraint_index += n;
  }

  // Check for positive definite Hessian matrix.
  Eigen::LLT<Eigen::MatrixXd> llt(G);
  if (llt.info() == Eigen::Success) {
    // Matrix is positive definite. (inv(G)*A')' = A*inv(G) because G is
    // symmetric.
    Eigen::MatrixXd AiG_T = llt.solve(A.transpose());

    // Compute a full pivoting, QR factorization.
    Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr(A * AiG_T);

    // Solve using least-squares A*inv(G)*A'y = A*inv(W)*c + b for `y`.
    Eigen::VectorXd lambda = qr.solve(AiG_T.transpose() * c + b);

    // Solve G*x = A'y - c
    prog.SetDecisionVariableValues(llt.solve(A.transpose() * lambda - c));
    prog.SetSolverResult("Equality Constrained QP Solver", 0);
    return SolutionResult::kSolutionFound;
  }

  // The following code assumes that the Hessian is not positive definite.
  // It uses the singular value decomposition, which is generally overkill but
  // does provide a useful fallback in the case that the range space approach
  // above fails.

  // The expanded problem introduces a Lagrangian multiplier for each
  // linear equality constraint.
  size_t num_full_vars = prog.num_vars() + num_constraints;
  Eigen::MatrixXd A_full(num_full_vars, num_full_vars);
  Eigen::VectorXd b_full(num_full_vars);

  // Set up the big matrix.
  A_full.block(0, 0, G.rows(), G.cols()) = G;
  A_full.block(0, G.cols(), A.cols(), A.rows()) = -A.transpose();
  A_full.block(G.rows(), 0, A.rows(), A.cols()) = A;
  A_full.block(G.rows(), G.cols(), A.rows(), A.rows()).setZero();

  // Set up the right hand side vector.
  b_full.segment(0, G.rows()) = -c;
  b_full.segment(G.rows(), A.rows()) = b;

  // Compute the least-squares solution.
  Eigen::VectorXd sol =
      A_full.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_full);
  prog.SetDecisionVariableValues(sol.segment(0, prog.num_vars()));

  prog.SetSolverResult(SolverName(), 0);
  return SolutionResult::kSolutionFound;
}

}  // namespace solvers
}  // namespace drake
