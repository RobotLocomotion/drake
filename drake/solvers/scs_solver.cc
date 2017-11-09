#include "drake/solvers/scs_solver.h"

#include <Eigen/Sparse>

#include "linsys/amatrix.h"
#include "scs.h"

#include "drake/common/text_logging.h"
#include "drake/math/eigen_sparse_triplet.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace solvers {
void ParseLinearCost(const MathematicalProgram& prog, Eigen::VectorXd* c,
                     double* constant) {
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

void ParseLinearEqualityConstraint(
    const MathematicalProgram& prog,
    std::vector<Eigen::Triplet<double>>* A_triplets, Eigen::VectorXd* b,
    int* A_row_count, SCS_CONE* cone) {
  int num_linear_equality_constraints_rows = 0;
  // The linear equality constraint A x = b is converted to
  // A x + s = b. s in zero cone.
  for (const auto& linear_equality_constraint :
       prog.linear_equality_constraints()) {
    const Eigen::SparseMatrix<double> Ai =
        linear_equality_constraint.constraint()->GetSparseMatrix();
    const std::vector<Eigen::Triplet<double>> Ai_triplets =
        math::SparseMatrixToTriplets(Ai);
    A_triplets->reserve(A_triplets->size() + Ai_triplets.size());
    const solvers::VectorXDecisionVariable& x =
        linear_equality_constraint.variables();
    // x_indices[i] is the index of x(i)
    std::vector<int> x_indices(x.rows());
    for (int i = 0; i < x.rows(); ++i) {
      x_indices[i] = prog.FindDecisionVariableIndex(x(i));
    }
    for (const auto& Ai_triplet : Ai_triplets) {
      A_triplets->emplace_back(Ai_triplet.row() + *A_row_count,
                               x_indices[Ai_triplet.col()], Ai_triplet.value());
    }
    const int num_Ai_rows =
        linear_equality_constraint.constraint()->num_constraints();
    b->resize(b->rows() + num_Ai_rows);
    b->bottomRows(num_Ai_rows) =
        linear_equality_constraint.constraint()->lower_bound();
    *A_row_count += num_Ai_rows;
    num_linear_equality_constraints_rows += num_Ai_rows;
  }
  cone->f += num_linear_equality_constraints_rows;
}

void ParseBoundingBoxConstraint(const MathematicalProgram& prog,
                                std::vector<Eigen::Triplet<double>>* A_triplets,
                                Eigen::VectorXd* b, int* A_row_count,
                                SCS_CONE* cone) {
  // A bounding box constraint l <= x <= u is converted to the SCS form as
  // x + s1 = u, -x + s2 = -l, s1, s2 in the positive cone.
  // Here we assume that u > l.
  // TODO(hongkai.dai) : handle the special case l = u, such that we can convert
  // it to x + s = l, s in zero cone.
  int num_bounding_box_constraint_rows = 0;
  for (const auto& bounding_box_constraint : prog.bounding_box_constraints()) {
    const VectorXDecisionVariable& xi = bounding_box_constraint.variables();
    const int num_xi_rows = xi.rows();
    A_triplets->reserve(A_triplets->size() + 2 * num_xi_rows);
    for (int i = 0; i < num_xi_rows; ++i) {
      A_triplets->emplace_back(i + *A_row_count, prog.FindDecisionVariableIndex(xi(i)), 1);
      A_triplets->emplace_back(i + *A_row_count + num_xi_rows, prog.FindDecisionVariableIndex(xi(i)), -1);
    }
    b->resize(b->rows() + 2 * num_xi_rows);
    b->bottomRows(2 * num_xi_rows) << bounding_box_constraint.constraint()->upper_bound(), -bounding_box_constraint.constraint()->lower_bound();
    *A_row_count += 2 * num_xi_rows;
    num_bounding_box_constraint_rows += 2 * num_xi_rows;
  }
  cone->l += num_bounding_box_constraint_rows;
}

std::string Scs_return_info(scs_int scs_status) {
  switch (scs_status) {
    case SCS_INFEASIBLE_INACCURATE:
      return "SCS infeasible inaccurate";
    case SCS_UNBOUNDED_INACCURATE:
      return "SCS unbounded inaccurate";
    case SCS_SIGINT:
      return "SCS sigint";
    case SCS_FAILED:
      return "SCS failed";
    case SCS_INDETERMINATE:
      return "SCS indeterminate";
    case SCS_INFEASIBLE:
      return "SCS primal infeasible, dual unbounded";
    case SCS_UNBOUNDED:
      return "SCS primal unbounded, dual infeasible";
    case SCS_UNFINISHED:
      return "SCS unfinished";
    case SCS_SOLVED:
      return "SCS solved";
    case SCS_SOLVED_INACCURATE:
      return "SCS solved inaccurate";
  }
}

void ExtractSolution(MathematicalProgram* prog,
                     const SCS_SOL_VARS& scs_sol_vars) {
  // For zero, positive or second-order cones, the primal variable x is the
  // same variable as in `prog`. For semidefinite cones, the variable x is a
  // scaled version of the lower-triangular part of the variable in `prog`, with
  // a scaling factor of √2.
  prog->SetDecisionVariableValues(
      Eigen::Map<Eigen::VectorXd>(scs_sol_vars.x, prog->num_vars()));
}

SolutionResult ScsSolver::Solve(MathematicalProgram& prog) const {
  // SCS solves the problem in this form
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
  // details on the cone types.
  // Notice that due to the special problem form supported by SCS, we need to
  // convert our generic constraints to SCS form. For example, a linear
  // inequality constraint
  //   A x ≤ b
  // will be converted as
  //   A x + s = b
  //   s in positive orthant cone.
  // by introducing the slack variable s.
  const int num_vars = prog.num_vars();

  // We need to construct a sparse matrix in Column Compressed Storage (CCS)
  // format. On the other hand, we add the constraint row by row, instead of
  // column by column, as preferred by CCS. As a result, we use Eigen sparse
  // matrix to construct a sparse matrix in column order first, and then
  // compress it to get CCS.
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

  // Parse linear cost
  ParseLinearCost(prog, &c, &cost_constant);

  // Parse linear equality constraint
  ParseLinearEqualityConstraint(prog, &A_triplets, &b, &A_row_count, &cone);

  // Parse bounding box constraint
  ParseBoundingBoxConstraint(prog, &A_triplets, &b, &A_row_count, &cone);

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

  scs_int scs_status =
      scs_solve(scs_work, &scs_problem_data, &cone, &scs_sol, &scs_info);

  SolutionResult sol_result{SolutionResult::kUnknownError};
  if (scs_status == SCS_SOLVED || scs_status == SCS_SOLVED_INACCURATE) {
    sol_result = SolutionResult::kSolutionFound;
    ExtractSolution(&prog, scs_sol);
  } else if (scs_status == SCS_UNBOUNDED ||
             scs_status == SCS_UNBOUNDED_INACCURATE) {
    sol_result = SolutionResult::kUnbounded;
  } else if (scs_status == SCS_INFEASIBLE ||
             scs_status == SCS_INFEASIBLE_INACCURATE) {
    sol_result = SolutionResult::kInfeasibleConstraints;
  }
  if (scs_status != SCS_SOLVED) {
    drake::log()->info("SCS returns code {}, with message \"{}\".\n",
                       scs_status, Scs_return_info(scs_status));
  }

  // Free allocated memory
  scs_finish(scs_work);
  return sol_result;
}
}  // namespace solvers
}  // namespace drake
