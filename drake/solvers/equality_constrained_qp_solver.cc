#include "drake/solvers/equality_constrained_qp_solver.h"

#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/never_destroyed.h"
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
  // minimize 1/2 x'*G*x + c'*x + constant_term
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
  double constant_term{0};
  for (auto const& binding : prog.quadratic_costs()) {
    const auto& Q = binding.constraint()->Q();
    const auto& b = binding.constraint()->b();
    constant_term += binding.constraint()->c();
    int num_v_variables = binding.variables().rows();

    std::vector<size_t> v_index(num_v_variables);
    for (int i = 0; i < num_v_variables; ++i) {
      v_index[i] = prog.FindDecisionVariableIndex(binding.variables()(i));
    }
    for (int i = 0; i < num_v_variables; ++i) {
      for (int j = 0; j < num_v_variables; ++j) {
        G(v_index[i], v_index[j]) += Q(i, j);
      }
      c(v_index[i]) += b(i);
    }
  }

  Eigen::VectorXd x{};
  SolutionResult solver_result{};
  if (num_constraints > 0) {
    // Setup the linear constraints.
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, prog.num_vars());
    Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints);
    int constraint_index = 0;
    for (auto const &binding : prog.linear_equality_constraints()) {
      auto const &bc = binding.constraint();
      size_t n = bc->A().rows();

      int num_v_variables = binding.variables().rows();
      for (int i = 0; i < num_v_variables; ++i) {
        A.block(constraint_index,
                prog.FindDecisionVariableIndex(binding.variables()(i)), n, 1) =
            bc->A().col(i);
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
      x = llt.solve(A.transpose() * lambda - c);
      solver_result = SolutionResult::kSolutionFound;
    } else {
      // The following code assumes that the Hessian is not positive definite.
      // It uses the singular value decomposition, which is generally overkill
      // but does provide a useful fallback in the case that the range space
      // approach above fails.

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
          A_full.jacobiSvd(
              Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_full);
      x = sol.segment(0, prog.num_vars());
      solver_result = SolutionResult::kSolutionFound;
    }
  } else {
    // num_constraints = 0
    // Check for positive definite Hessian matrix.
    Eigen::LLT<Eigen::MatrixXd> llt(G);
    if (llt.info() == Eigen::Success) {
      // G is positive definite.
      x = llt.solve(-c);
      solver_result = SolutionResult::kSolutionFound;
    } else {
      // G is not strictly positive definite.
      // There are two possible cases
      // 1. If there exists x, s.t G * x = -c, and G is positive semidefinite
      //    then there are infinitely many optimal solutions, and we will return
      //    one of the optimal solutions.
      // 2. Otherwise, if G * x = -c does not have a solution, or G is not
      //    positive semidefinite, then the problem is unbounded.

      // We first check if G is positive semidefinite, by doing an LDLT
      // decomposition.
      Eigen::LDLT<Eigen::MatrixXd> ldlt(G);
      if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
        x = Eigen::VectorXd::Constant(prog.num_vars(), NAN);
        return SolutionResult::kUnbounded;
      }
      // G is positive semidefinite.
      x = ldlt.solve(-c);
      // The precision 1E-10 here is random.
      if (!(G * x).isApprox(-c, 1E-10)) {
        x = Eigen::VectorXd::Constant(prog.num_vars(), NAN);
        solver_result = SolutionResult::kUnbounded;
      } else {
        solver_result = SolutionResult::kSolutionFound;
      }
    }
  }

  prog.SetDecisionVariableValues(x);
  double optimal_cost{};
  if (solver_result == SolutionResult::kSolutionFound) {
    optimal_cost = 0.5 * x.dot(G * x) + c.dot(x) + constant_term;
  } else if (solver_result == SolutionResult::kUnbounded) {
    optimal_cost = -std::numeric_limits<double>::infinity();
  } else {
    optimal_cost = NAN;
  }
  prog.SetOptimalCost(optimal_cost);
  prog.SetSolverId(id());
  return solver_result;
}

SolverId EqualityConstrainedQPSolver::solver_id() const {
  return id();
}

SolverId EqualityConstrainedQPSolver::id() {
  static const never_destroyed<SolverId> singleton{"Equality constrained QP"};
  return singleton.access();
}

}  // namespace solvers
}  // namespace drake
