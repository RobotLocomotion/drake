#include "drake/solvers/equality_constrained_qp_solver.h"

#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_optional.h"
#include "drake/common/never_destroyed.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {

bool EqualityConstrainedQPSolver::available() const { return true; }

/**
 * Solves the un-constrained QP problem
 * min 0.5 * xᵀ * G * x + cᵀ * x
 */
SolutionResult SolveUnconstrainedQP(const Eigen::Ref<const Eigen::MatrixXd>& G,
                                    const Eigen::Ref<const Eigen::VectorXd>& c,
                                    double feasibility_tol,
                                    Eigen::VectorXd* x) {
  // If the Hessian G is positive definite, then the problem has a unique
  // optimal solution.
  // If the Hessian G has one or more negative eigen values, then the problem is
  // unbounded.
  // If the Hessian G is positive semidefinite, but with some eigen values
  // being 0, then we check the first order derivative G * x + c. If there
  // exists solution x* such that the first order derivative at x* is 0, then
  // the problem has optimal cost (but infinitely many optimal x* will exist).

  // Check for positive definite Hessian matrix.
  Eigen::LLT<Eigen::MatrixXd> llt(G);
  if (llt.info() == Eigen::Success) {
    // G is positive definite.
    *x = llt.solve(-c);
    return SolutionResult::kSolutionFound;
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
    if (ldlt.info() == Eigen::Success && ldlt.isPositive()) {
      // G is positive semidefinite.
      *x = ldlt.solve(-c);
      if ((G * (*x)).isApprox(-c, feasibility_tol)) {
        return SolutionResult::kSolutionFound;
      }
    }
    *x = Eigen::VectorXd::Constant(c.rows(), NAN);
    return SolutionResult::kUnbounded;
  }
}

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

  // The QP approach attempts Approach (2), if G is strictly positive definite,
  // and falls back to Approach (3) otherwise. For the null-space approach, we
  // compute kernel(A) = N, and convert the equality constrained QP to an
  // un-constrained QP, as
  // minimize 0.5 y'*(N'*G*N)*y + (c'*N + N'*G*x0)* y
  // where x0 is one solution to A * x = b.
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

  // The default tolerance is Eigen's dummy precision.
  double feasibility_tol = Eigen::NumTraits<double>::dummy_precision();
  std::map<std::string, double> option_double =
      prog.GetSolverOptionsDouble(EqualityConstrainedQPSolver::id());
  auto it = option_double.find("FeasibilityTol");
  if (it != option_double.end()) {
    if (it->second >= 0) {
      feasibility_tol = it->second;
      option_double.erase(it);
    } else {
      throw std::runtime_error(
          "FeasibilityTol should be a non-negative number.");
    }
  }
  if (!option_double.empty()) {
    throw std::runtime_error(
        "Unsupported option in EqualityConstrainedQPSolver.");
  }

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
  for (const auto& binding : prog.linear_costs()) {
    const auto& a = binding.constraint()->a();
    constant_term += binding.constraint()->b();
    int num_v_variables = binding.variables().rows();

    for (int i = 0; i < num_v_variables; ++i) {
      auto v_index_i = prog.FindDecisionVariableIndex(binding.variables()(i));
      c(v_index_i) += a(i);
    }
  }

  Eigen::VectorXd x{};
  optional<SolutionResult> solver_result;
  if (num_constraints > 0) {
    // Setup the linear constraints.
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, prog.num_vars());
    Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints);
    int constraint_index = 0;
    for (auto const& binding : prog.linear_equality_constraints()) {
      auto const& bc = binding.constraint();
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
      const Eigen::MatrixXd A_iG_A_T = A * AiG_T;
      Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr(A_iG_A_T);

      // Solve using least-squares A*inv(G)*A'y = A*inv(G)*c + b for `y`.
      const Eigen::VectorXd rhs = AiG_T.transpose() * c + b;
      Eigen::VectorXd lambda = qr.solve(rhs);

      solver_result = rhs.isApprox(A_iG_A_T * lambda, feasibility_tol)
                          ? SolutionResult::kSolutionFound
                          : SolutionResult::kInfeasibleConstraints;

      // Solve G*x = A'y - c
      x = llt.solve(A.transpose() * lambda - c);
    } else {
      // The following code assumes that the Hessian is not positive definite.
      // We first compute the null space of A. Denote kernel(A) = N.
      // If A * x = b is feasible, then x = x₀ + N * y, where A * x₀ = b.
      // The QP can be re-formulated as an un-constrained QP problem
      // min 0.5 * (x₀ + N * y)ᵀ * G * (x₀ + N * y) + cᵀ * (x₀ + N * y)
      // which has the same optimal solution as
      // min 0.5 * yᵀ * Nᵀ*G*N * y + (x₀ᵀ*G*N + cᵀ*N) * y
      Eigen::JacobiSVD<Eigen::MatrixXd> svd_A_thin(
          A, Eigen::ComputeThinU | Eigen::ComputeThinV);
      const Eigen::VectorXd x0 = svd_A_thin.solve(b);
      if (!b.isApprox(A * x0, feasibility_tol)) {
        solver_result = SolutionResult::kInfeasibleConstraints;
        x = x0;
      } else {
        if (svd_A_thin.rank() == A.cols()) {
          // The kernel is empty, the solution is unique.
          solver_result = SolutionResult::kSolutionFound;
          x = x0;
        } else {
          // N is the null space of A
          // Using QR decomposition
          // Aᵀ * P = [Q1 Q2] * [R] = Q1 * R
          //                    [0]
          // where P is a permutation matrix.
          // So A = P * R1ᵀ * Q1ᵀ, and A * Q2 = P * R1ᵀ * Q1ᵀ * Q2 = 0 since
          // Q1 and Q2 are orthogonal to each other.
          // Thus kernel(A) = Q2.
          // Notice that we do not call svd here because svd only gives
          // us a "thin" V; thus the V matrix does not contain the basis vectors
          // for the null space.
          Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_A(A.transpose());
          const Eigen::MatrixXd Q =
              qr_A.householderQ().setLength(qr_A.nonzeroPivots());
          const Eigen::MatrixXd N = Q.rightCols(A.cols() - qr_A.rank());
          Eigen::VectorXd y(N.cols());
          solver_result = SolveUnconstrainedQP(
              N.transpose() * G * N, x0.transpose() * G * N + c.transpose() * N,
              feasibility_tol, &y);
          x = x0 + N * y;
        }
      }
    }
  } else {
    // num_constraints = 0
    solver_result = SolveUnconstrainedQP(G, c, feasibility_tol, &x);
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
  // Make sure solver_result is set.
  DRAKE_DEMAND(!!solver_result);
  return *solver_result;
}

SolverId EqualityConstrainedQPSolver::solver_id() const { return id(); }

SolverId EqualityConstrainedQPSolver::id() {
  static const never_destroyed<SolverId> singleton{"Equality constrained QP"};
  return singleton.access();
}

}  // namespace solvers
}  // namespace drake
