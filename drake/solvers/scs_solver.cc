#include "drake/solvers/scs_solver.h"

#include <Eigen/Sparse>

#include "scs.h"
#include "linsys/amatrix.h"

#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
void ParseLinearCost(const MathematicalProgram& prog, Eigen::VectorXd* c, double* constant) {
  for (const auto& linear_cost : prog.linear_costs()) {
    // Each linear cost is in the form of aᵀx + b
    const auto& a = linear_cost.constraint()->a();
    const VectorXDecisionVariable& x = linear_cost.variables();
    for (int i = 0; i < a.rows(); ++i) {
      (*c)(prog.FindDecisionVariableIndex(x(i))) += a(i);
    }
    (*constant) += linear_cost.constraint()->b();
  }
}

void ParseLinearEqualityConstraint(const MathematicalProgram& prog, std::vector<Eigen::Triplet<double>>* A_triplets, Eigen::VectorXd* b, int* A_row_count, SCS_CONE* cone) {
  int num_linear_equality_constraints_rows = 0;
  // The linear equality constraint A x = b is converted to
  // A x + s = b. s in zero cone.
  for(const auto& linear_equality_constraint : prog.linear_equality_constraints()) {
    const Eigen::SparseMatrix<double> Ai = linear_equality_constraint.constraint()->GetSparseMatrix();
    const std::vector<Eigen::Triplet<double>> Ai_triplets = math::SparseMatrixToTriplets(Ai);
    A_triplets->reserve(A_triplets->size() + Ai_triplets.size());
    const solvers::VectorXDecisionVariable& x = linear_equality_constraint.variables();
    // x_indices[i] is the index of x(i)
    std::vector<int> x_indices(x.rows());
    for (int i = 0; i < x.rows(); ++i) {
      x_indices[i] = prog.FindDecisionVariableIndex(x(i));
    }
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(Ai_triplet.row() + *A_row_count, x_indices[Ai_triplet.col()], Ai_triplet.value());
    }
    const int num_Ai_rows = linear_equality_constraint.constraint()->num_constraints();
    b->resize(b->rows() + num_Ai_rows);
    b->bottomRows(num_Ai_rows) = linear_equality_constraint.constraint()->lower_bound();
    A_row_count += num_Ai_rows;
    num_linear_equality_constraints_rows += num_Ai_rows;
  }
  cone->f += num_linear_equality_constraints_rows;
}

SolutionResult ScsSolver::Solve(MathematicalProgram& prog) const {
  // SCS solves the problem in this format
  // min  cᵀx
  // s.t A x + s = b
  //     s in K
  // where K is a Cartesian product of some primitive cones.
  // The cones has to be in this order
  // Zero cone {x | x = 0 }
  // Positive orthant {x | x ≥ 0 }
  // Second-order cone {(t, x) | |x|₂ ≤ t }
  // Positive semidefinite cone { X | min(eig(X)) ≥ 0, X = Xᵀ }
  // There are more types that are supported by SCS, after the Positive
  // semidefinite cone. Please refer to https://github.com/cvxgrp/scs for more
  // details on the type of cones.
  // Notice that due to the special problem form supported by SCS, we need to
  // convert our generic constraints to SCS form. For example, a linear
  // inequality constraint
  //   A x ≤ b
  // will be converted as
  //   A x + s = b
  //   s in positive orthant cone.
  // by introducing the slack variable s.
  const int num_vars = prog.num_vars();

  // We need to construct a sparse matrix in the Column Compressed Storage (CCS).
  // On the other hand, we add the constraint row by row, instead of column by
  // column, as preferred by CCS. As a result, we use Eigen sparse matrix to
  // construct a sparse matrix in column order first, and then compress it to
  // get CCS.
  std::vector<Eigen::Triplet<double>> A_triplets;

  // cone stores all the cones K in the problem.
  SCS_CONE cone;
  cone.f = 0;
  cone.l = 0;
  cone.q = nullptr;
  cone.qsize = 0;
  cone.s = nullptr;
  cone.ssize = 0;
  cone.ep = 0;
  cone.ed = 0;
  cone.p = nullptr;
  cone.psize = 0;
  // A_row_count will increment, when we add each constraint.
  int A_row_count = 0;
  Eigen::VectorXd b(0);

  Eigen::VectorXd c = Eigen::VectorXd::Zero(num_vars);
  double cost_constant{0};
  ParseLinearCost(prog, &c, &cost_constant);

  ParseLinearEqualityConstraint(prog, &A_triplets, &b, &A_row_count, &cone);

  SCS_PROBLEM_DATA scs_problem_data;
  scs_problem_data.m = A_row_count;
  scs_problem_data.n = num_vars;
  Eigen::SparseMatrix<double> A(scs_problem_data.m, scs_problem_data.n);
  A.setFromTriplets(A_triplets.begin(), A_triplets.end());
  A.makeCompressed();
  scs_problem_data.A->x = A.valuePtr();
  scs_problem_data.A->i = A.innerIndexPtr();
  scs_problem_data.A->p = A.outerIndexPtr();
  scs_problem_data.A->m = scs_problem_data.m;
  scs_problem_data.A->n = scs_problem_data.n;
  scs_problem_data.b = b.data();

  SCS_INFO scs_info;
  SCS_WORK* scs_work = scs_init(&scs_problem_data, &cone, &scs_info);

  SCS_SOL_VARS scs_sol;

  scs_int scs_status = scs_solve(scs_work, &scs_problem_data, &cone, &scs_sol, &scs_info);

  SolutionResult sol_result{SolutionResult::kUnknownError};
  if (scs_status == SCS_SOLVED) {
    sol_result = SolutionResult::kSolutionFound;
  }

  return sol_result;
}
}  // namespace solvers
}  // namespace drake
